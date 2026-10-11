"""Loopback-only web bridge for owned SITL runs. No hardware endpoints or actuator API."""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time
from urllib.parse import urlsplit

from .transport import ROOT
from .demo import FAULTS
from quadstack.sensors import DELIVERY_SCENARIOS
from .geographic_flight import resolve_request

DIST = ROOT/"site/flight-ops/dist"
PYTHON = ROOT/".venv/bin/python"
SCENARIOS = frozenset((*DELIVERY_SCENARIOS, *FAULTS))
SITE_ORIGIN = "https://quad-flight-ops.validokapi9.chatgpt.site"


class Supervisor:
    def __init__(self):
        self.token = secrets.token_urlsafe(24)
        self.process = None
        self.directory = None
        self.kind = None
        self.started = None
        self.log = None
        self.lock = threading.Lock()

    def launch(self, command, kind):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                raise ValueError("a SITL run is already active")
            if self.log is not None:
                self.log.close()
            self.directory = ROOT/"simulation/runs/flight-ops"/(time.strftime("%Y%m%d-%H%M%S")+"-"+secrets.token_hex(3))
            self.directory.mkdir(parents=True)
            self.log = (self.directory/"console.log").open("w")
            env = dict(os.environ, PYTHONPATH=str(ROOT/"flight-software"))
            self.process = subprocess.Popen([str(PYTHON), *command, "--output", str(self.directory)],
                                            cwd=ROOT, env=env, stdout=self.log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
            self.kind, self.started = kind, time.time()

    def state(self):
        if self.process is None:
            return {"status": "IDLE", "run": None, "telemetry": {}, "track": [], "events": []}
        code = self.process.poll()
        result_path = self.directory/"result.json"
        result = None
        if result_path.exists():
            try:
                result = json.loads(result_path.read_text())
            except (OSError, ValueError):
                pass
        telemetry, track, events = {}, [], []
        path = self.directory/"telemetry.jsonl"
        if path.exists():
            with path.open("rb") as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell()-1_000_000))
                if stream.tell():
                    stream.readline()
                for raw in stream:
                    try:
                        item = json.loads(raw)
                    except ValueError:
                        continue
                    kind = item.get("mavpackettype")
                    if kind:
                        telemetry[kind] = item
                    if kind == "GLOBAL_POSITION_INT":
                        track.append([item.get("lat"), item.get("lon"), item.get("relative_alt"), item.get("wall")])
                    elif kind in ("STATUSTEXT", "MISSION_ITEM_REACHED"):
                        events.append(item)
        log_tail = ""
        if code is not None and self.log:
            self.log.flush()
            with (self.directory/"console.log").open("rb") as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell()-2000))
                log_tail = stream.read().decode(errors="replace")
        route_path = self.directory/"route.json"
        phase_path = self.directory/"phase.json"
        try:
            route = json.loads(route_path.read_text()) if route_path.exists() else None
        except (OSError, ValueError):
            route = None
        try:
            phase = json.loads(phase_path.read_text()) if phase_path.exists() else None
        except (OSError, ValueError):
            phase = None
        return {"status": "RUNNING" if code is None else ("PASS" if code == 0 and result and result.get("status") == "PASS" else "FAIL"),
                "kind": self.kind, "started": self.started, "run": self.directory.name,
                "telemetry": telemetry, "track": track[-600:], "events": events[-30:],
                "telemetry_age_s": max(0,time.monotonic()-telemetry["GLOBAL_POSITION_INT"]["wall"])
                                   if "GLOBAL_POSITION_INT" in telemetry else None,
                "result": result, "route": route, "phase": phase, "console": log_tail, "exit_code": code}


supervisor = Supervisor()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIST), **kwargs)

    def allowed_origin(self):
        origin = self.headers.get("Origin")
        return origin if origin in (SITE_ORIGIN, "http://127.0.0.1:8765", "http://localhost:8765",
                                    "http://127.0.0.1:5500", "http://localhost:5500") else None

    def end_headers(self):
        if origin := self.allowed_origin():
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Flight-Token")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204 if self.allowed_origin() else 403)
        self.end_headers()

    def response(self, status, value):
        data = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def authorized(self):
        return secrets.compare_digest(self.headers.get("X-Flight-Token", ""), supervisor.token)

    def do_GET(self):
        if urlsplit(self.path).path == "/api/state":
            if not self.authorized():
                return self.response(401, {"error": "bridge access code required"})
            return self.response(200, supervisor.state())
        if self.path.startswith("/api/"):
            return self.response(404, {"error": "unknown endpoint"})
        super().do_GET()

    def do_POST(self):
        if not self.authorized():
            return self.response(401, {"error": "bridge access code required"})
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            return self.response(415, {"error": "JSON required"})
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 2048:
                raise ValueError("invalid request size")
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError("request must be an object")
            path = urlsplit(self.path).path
            if path == "/api/position":
                if set(body) != {"north", "east", "up", "speed"}:
                    raise ValueError("expected north, east, up, speed")
                values = [body[key] for key in ("north", "east", "up", "speed")]
                if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                    raise ValueError("coordinates and speed must be finite numbers")
                north, east, up, speed = values
                if math.hypot(north, east) > 80 or math.hypot(north, east) < 3 or not 3 <= up <= 15 or not .5 <= speed <= 5:
                    raise ValueError("outside SITL position/speed envelope")
                supervisor.launch(["-m", "simulation.fly_to", "--north", str(north), "--east", str(east),
                                   "--up", str(up), "--speed", str(speed)], "position")
            elif path == "/api/geographic":
                route = resolve_request(body)
                if route.budget_wh > 222:
                    raise ValueError("route exceeds assumed fully charged energy budget; increase speed or shorten route")
                supervisor.launch(["-m", "simulation.geographic_flight", "--request", json.dumps(body)], "geographic")
            elif path == "/api/scenario":
                if set(body) != {"scenario", "enable_release"} or body["scenario"] not in SCENARIOS or type(body["enable_release"]) is not bool:
                    raise ValueError("unknown test scenario")
                if body["scenario"] in {"jammed_latch", "failed_release"} and not body["enable_release"]:
                    raise ValueError("this scenario requires explicit in-memory release enablement")
                command = ["-m", "simulation.demo", "--scenario", body["scenario"], "--sitl-flight"]
                if body["enable_release"]:
                    command.append("--enable-simulated-release")
                supervisor.launch(command, body["scenario"])
            else:
                return self.response(404, {"error": "unknown endpoint"})
            return self.response(202, {"status": "started", "run": supervisor.directory.name})
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            return self.response(400, {"error": str(error)})


def main():
    if not PYTHON.exists() or not (ROOT/".tools/ardupilot/build/sitl/bin/arducopter").exists():
        raise SystemExit("SITL toolchain missing; run bash simulation/setup.sh")
    server = ThreadingHTTPServer(("127.0.0.1", 8765), Handler)
    print("Flight Ops bridge: http://127.0.0.1:8765", flush=True)
    print(f"Bridge access code: {supervisor.token}", flush=True)
    print("SITL only; no physical vehicle or actuator endpoint", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

"""Owned local SITL session. Never accepts a serial path, host, or external endpoint."""
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import socket
import subprocess
import time
from pymavlink import mavutil
from quadstack.core import Point, Health, to_gps, from_gps

ROOT = Path(__file__).resolve().parents[1]
COMMIT = "92b0cd788ec29406f26c6f9c31d5ceedbd1cc538"
M = mavutil.mavlink
SAMPLED_TELEMETRY = frozenset(("GPS_RAW_INT", "SYS_STATUS", "EXTENDED_SYS_STATE",
                               "GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD",
                               "SERVO_OUTPUT_RAW", "RC_CHANNELS", "BATTERY_STATUS",
                               "EKF_STATUS_REPORT", "DISTANCE_SENSOR"))
EVENT_TELEMETRY = frozenset(("HEARTBEAT", "STATUSTEXT", "MISSION_ITEM_REACHED",
                             "MISSION_CURRENT", "COMMAND_ACK", "MISSION_ACK"))


def udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def heartbeat_service(port, stop, parent_pid, lease):
    """Companion health process, sole owner of SYSID_MYGCS heartbeats.

    The parent supervisor owns authenticated mission policy. Parent death also stops
    this service; an orphan cannot mask a failed coordinator with healthy heartbeats.
    """
    connection = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=255, source_component=191)
    last = 0.
    last_received = time.monotonic()
    while not stop.is_set() and os.getppid() == parent_pid:
        message = connection.recv_match(blocking=False)
        if message is not None and message.get_srcSystem() == 1:
            last_received = time.monotonic()
        if time.monotonic()-last > .1 and time.monotonic()-last_received < 2 and time.monotonic()-lease.value < 2:
            connection.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last = time.monotonic()
        time.sleep(.01)
    connection.close()


class Link:
    def __init__(self, connection, session):
        self.connection, self.session = connection, session
        self.messages = {}
        self.received = {}
        self.events = []
        self.logged = {}
        self.mode = "UNKNOWN"
        self.armed = False
        self.sim_time = 0.
        self.connection.mav.set_send_callback(self.sent)

    def sent(self, message):
        with (self.session.directory/"outgoing.jsonl").open("a") as f:
            f.write(json.dumps(dict(wall=time.monotonic(),**message.to_dict()))+"\n")

    def recv(self, timeout=.1):
        self.session.lease.value = time.monotonic()
        if self.session.process.poll() is not None:
            raise RuntimeError("owned SITL process exited")
        msg = self.connection.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
            return None
        name = msg.get_type()
        self.messages[name] = msg
        self.received[name] = time.monotonic()
        if hasattr(msg, "time_boot_ms"):
            self.sim_time = msg.time_boot_ms/1000.
        if name == "HEARTBEAT":
            self.mode = mavutil.mode_string_v10(msg)
            self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
        if name in SAMPLED_TELEMETRY and time.monotonic()-self.logged.get(name, 0) < .5:
            return msg
        if name in EVENT_TELEMETRY or name in SAMPLED_TELEMETRY:
            self.logged[name] = time.monotonic()
            event = dict(wall=time.monotonic(), sim=self.sim_time, **msg.to_dict())
            self.events.append(event)
            with self.session.telemetry.open("a") as f:
                f.write(json.dumps(event)+"\n")
        return msg

    def wait(self, predicate, timeout=30, description="condition"):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            msg = self.recv()
            if predicate(msg):
                return msg
        recent = [e for e in self.events if e.get("mavpackettype") == "STATUSTEXT"][-8:]
        raise TimeoutError(f"{description}; mode={self.mode}, armed={self.armed}; status={recent}")

    def drain(self, seconds=.1):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.recv(.01)

    @property
    def position(self):
        p = self.messages.get("GLOBAL_POSITION_INT")
        if p and self.session.geographic_route:
            return self.session.geographic_route.local(p.lat/1e7, p.lon/1e7, p.relative_alt/1000.)
        return from_gps(p.lat/1e7, p.lon/1e7, p.relative_alt/1000.) if p else Point(0,0,0)

    @property
    def landed(self):
        e = self.messages.get("EXTENDED_SYS_STATE")
        return e is not None and e.landed_state == M.MAV_LANDED_STATE_ON_GROUND

    def health(self):
        """Explicit stock-3S -> conceptual-6S adapter, not a physical measurement.

        Percentage maps to assumed 222 Wh nameplate for policy exercises only.
        FC failsafes independently use their stock-model voltage thresholds.
        """
        now = time.monotonic()
        battery = self.messages.get("SYS_STATUS")
        gps = self.messages.get("GPS_RAW_INT")
        fresh = all(now-self.received.get(n,0) < 1 for n in ("HEARTBEAT","SYS_STATUS","GPS_RAW_INT","GLOBAL_POSITION_INT","EXTENDED_SYS_STATE"))
        return Health(timestamp=now if fresh else now-3,
                      battery_wh=max(0,battery.battery_remaining)*2.22 if battery else 0,
                      voltage=battery.voltage_battery/1000.*2 if battery else 0,
                      gnss_ok=gps is not None and gps.fix_type>=3 and gps.satellites_visible>=6,
                      telemetry_ok=fresh, companion_ok=self.session.health_process.is_alive(),
                      rc_ok=bool(battery and battery.onboard_control_sensors_health & M.MAV_SYS_STATUS_SENSOR_RC_RECEIVER),
                      estimator_ok=bool(battery and battery.onboard_control_sensors_health & M.MAV_SYS_STATUS_AHRS))

    def parameter(self, name, value=None):
        # Test harness only; the companion API does not expose this capability.
        if value is not None and name not in ("SIM_BATT_VOLTAGE", "SIM_GPS_DISABLE", "SIM_GPS_NUMSATS", "SIM_RC_FAIL"):
            raise ValueError("only allowlisted simulator fault injection is writable")
        for attempt in range(3):
            if value is None:
                self.connection.mav.param_request_read_send(1,1,name.encode(),-1)
            else:
                self.connection.mav.param_set_send(1,1,name.encode(),float(value),M.MAV_PARAM_TYPE_REAL32)
            try:
                msg = self.wait(lambda m: m is not None and m.get_type()=="PARAM_VALUE" and m.param_id==name, 3, f"parameter {name}")
                if value is not None and abs(msg.param_value-value) > .001:
                    raise AssertionError((name, value, msg.param_value))
                return msg.param_value
            except TimeoutError:
                if attempt == 2:
                    raise

    def command(self, command, *params):
        if command not in (M.MAV_CMD_SET_MESSAGE_INTERVAL, M.MAV_CMD_COMPONENT_ARM_DISARM, M.MAV_CMD_MISSION_START):
            raise ValueError("command outside simulation harness allowlist")
        if command == M.MAV_CMD_COMPONENT_ARM_DISARM and (not self.session.flight_enabled or params != (1,)):
            raise ValueError("only explicitly enabled normal SITL arming is allowed")
        params = list(params)+[0.]*(7-len(params))
        self.connection.mav.command_long_send(1,1,command,0,*params)
        ack = self.wait(lambda m: m is not None and m.get_type()=="COMMAND_ACK" and m.command==command, 5, f"command {command} ACK")
        if ack.result not in (M.MAV_RESULT_ACCEPTED, M.MAV_RESULT_IN_PROGRESS):
            raise RuntimeError(f"command {command} rejected: {ack.result}")

    def mode_request(self, mode):
        mode_id = self.connection.mode_mapping()[mode]
        self.connection.mav.set_mode_send(1,M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
        self.wait(lambda _: self.mode==mode, 8, f"mode {mode}")

    def request_recovery(self, mode):
        if mode not in ("RTL", "LAND"):
            raise ValueError("recovery mode not allowed")
        self.mode_request(mode)

    def upload(self, plan, returning=False):
        # Every uploaded mission starts with a home placeholder and explicit takeoff,
        # ends in LAND. The caller must separately opt into the simulator flight.
        points = list(plan.points)
        if returning:
            points = list(reversed(points[:-1]))+[Point(0,0,plan.points[0].up)]
        start = self.position
        items = [(M.MAV_CMD_NAV_WAYPOINT, Point(0,0,0)),
                 (M.MAV_CMD_NAV_TAKEOFF, Point(start.north,start.east,plan.points[0].up)),
                 (M.MAV_CMD_DO_CHANGE_SPEED, Point(0,0,0))]
        items += [(M.MAV_CMD_NAV_WAYPOINT,p) for p in points]
        last = points[-1]
        items += [(M.MAV_CMD_NAV_LAND,Point(last.north,last.east,0))]
        self.connection.mav.mission_count_send(1,1,len(items))
        end = time.monotonic()+20
        sent = set()
        while time.monotonic()<end:
            msg = self.recv()
            if msg is None:
                continue
            if msg.get_type() in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                i = msg.seq
                if not 0 <= i < len(items):
                    raise RuntimeError("invalid mission request sequence")
                command,p = items[i]
                lat,lon,alt = (self.session.geographic_route.global_position(p)
                               if self.session.geographic_route else to_gps(p))
                speed_item = command == M.MAV_CMD_DO_CHANGE_SPEED
                self.connection.mav.mission_item_int_send(1,1,i,M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,command,0,1,
                    1 if speed_item else 0, plan.speed if speed_item else 0, -1 if speed_item else 0, 0,
                    int(round(lat*1e7)),int(round(lon*1e7)),alt)
                sent.add(i)
            elif msg.get_type()=="MISSION_ACK":
                if msg.type!=M.MAV_MISSION_ACCEPTED or len(sent)!=len(items):
                    raise RuntimeError(f"mission upload failed: {msg}")
                return len(items)
        raise TimeoutError("mission upload handshake")

    def start_simulated_flight(self):
        if not self.session.flight_enabled:
            raise RuntimeError("requires explicit --sitl-flight")
        self.mode_request("GUIDED")
        # Normal arming: no force magic, no disabled checks.
        self.command(M.MAV_CMD_COMPONENT_ARM_DISARM,1)
        self.wait(lambda _: self.armed, 5, "simulated arming")
        self.mode_request("AUTO")
        self.command(M.MAV_CMD_MISSION_START,0,0)
        self.wait(lambda _: self.position.up>3, 20, "takeoff")


class SitlSession:
    def __init__(self, directory, *, flight_enabled=False, geographic_route=None):
        self.directory = Path(directory).resolve()
        self.flight_enabled = flight_enabled
        if geographic_route is not None:
            from quadstack.geographic import GeographicRoute
            if not isinstance(geographic_route, GeographicRoute):
                raise ValueError("expected validated geographic route")
            GeographicRoute.parse(dict(path=[list(p) for p in geographic_route.path],
                                       altitude=geographic_route.altitude, speed=geographic_route.speed))
        self.geographic_route = geographic_route
        self.process = None
        self.health_process = None
        self.connection = None
        self.log_file = None
        self.telemetry = self.directory/"telemetry.jsonl"

    def __enter__(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        self.telemetry.write_text("")
        (self.directory/"outgoing.jsonl").write_text("")
        source = ROOT/".tools/ardupilot"
        binary = source/"build/sitl/bin/arducopter"
        if not binary.is_file():
            raise RuntimeError("SITL not built; run bash simulation/setup.sh")
        actual = subprocess.check_output(["git","rev-parse","HEAD"],cwd=source,text=True).strip()
        if actual != COMMIT:
            raise RuntimeError("unexpected ArduPilot revision")
        # No endpoint argument. A fresh UDP receiver is created before the owned child.
        port, companion_port = udp_port(), udp_port()
        self.connection = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=254, source_component=190)
        ctx = multiprocessing.get_context("spawn")
        self.stop = ctx.Event()
        self.lease = ctx.Value("d",time.monotonic())
        self.health_process = ctx.Process(target=heartbeat_service,args=(companion_port,self.stop,os.getpid(),self.lease),daemon=True)
        self.health_process.start()
        defaults = f"{source}/Tools/autotest/default_params/copter.parm,{ROOT}/simulation/sitl.parm"
        home = "-35.363261,149.165230,584,0"
        if self.geographic_route:
            route = self.geographic_route
            home = f"{route.path[0][0]},{route.path[0][1]},100,0"
            # Generated only for this owned simulator. No safety check/failsafe overrides.
            overlay = self.directory/"geographic-sitl-only.parm"
            overlay.write_text(f"# SITL ONLY: full-distance Columbia navigation study\n"
                               f"FENCE_RADIUS {route.fence_radius}\nFENCE_ALT_MAX 100\n"
                               f"RTL_ALT {route.altitude*100}\nWPNAV_SPEED {route.speed*100}\n")
            defaults += f",{overlay}"
            (self.directory/"route.json").write_text(json.dumps(route.summary(),indent=2)+"\n")
        args = [str(binary),"-S","--model","x","--speedup","5","--home",home,
                "--defaults",defaults,"--wipe","--serial0",f"udpclient:127.0.0.1:{port}",
                "--serial1",f"udpclient:127.0.0.1:{companion_port}","--serial2","none",
                "--serial3","GPS1","--serial4","GPS2",
                "--serial5","none","--serial6","none","--serial7","none","--serial8","none",
                "--rc-in-port",str(udp_port())]
        self.log_file = (self.directory/"sitl.log").open("w")
        self.process = subprocess.Popen(args,cwd=self.directory,stdout=self.log_file,stderr=subprocess.STDOUT)
        manifest = dict(commit=actual, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                        model="ArduPilot stock X quad (model x, FRAME_TYPE=1)", speedup=5,
                        parameters_sha256=hashlib.sha256((ROOT/"simulation/sitl.parm").read_bytes()).hexdigest(),
                        command=[s.replace(str(ROOT),"<repo>") for s in args], hardware=False)
        (self.directory/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        if self.geographic_route:
            manifest["geographic_profile"] = self.geographic_route.summary()
            manifest["geographic_parameters_sha256"] = hashlib.sha256(overlay.read_bytes()).hexdigest()
            (self.directory/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        self.link = Link(self.connection,self)
        try:
            self.link.wait(lambda m: m is not None and m.get_type()=="HEARTBEAT",15,"SITL heartbeat")
            self.connection.mav.request_data_stream_send(1,1,M.MAV_DATA_STREAM_ALL,10,1)
            self.link.command(M.MAV_CMD_SET_MESSAGE_INTERVAL,M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,100000)
            self.link.wait(lambda _: self.link.messages.get("GPS_RAW_INT") is not None and self.link.messages["GPS_RAW_INT"].fix_type>=3 and self.link.landed,30,"GNSS and landed state")
            self.link.wait(lambda _: self.link.messages.get("SYS_STATUS") is not None and
                           self.link.messages["SYS_STATUS"].onboard_control_sensors_health & M.MAV_SYS_STATUS_PREARM_CHECK,
                           35, "flight controller pre-arm checks healthy")
            return self
        except BaseException:
            self.__exit__(None,None,None)
            raise

    def stop_companion(self, crash=False):
        if crash:
            self.health_process.kill()
        else:
            self.stop.set()
        self.health_process.join(3)

    def __exit__(self,*args):
        if self.health_process is not None:
            self.stop.set()
            self.health_process.join(2)
            if self.health_process.is_alive():
                self.health_process.terminate()
                self.health_process.join(2)
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.connection is not None:
            self.connection.close()
        if self.log_file is not None:
            self.log_file.close()

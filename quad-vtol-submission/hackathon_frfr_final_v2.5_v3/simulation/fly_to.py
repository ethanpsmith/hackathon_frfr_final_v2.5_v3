"""Run one bounded local-position round trip in an owned ArduCopter SITL child."""
import argparse
import json
import math
from pathlib import Path
import time

from quadstack.core import Coordinator, Obstacle, Rejected, State
from quadstack.service import CompanionService
from .transport import ROOT, SitlSession


def run(position, speed, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    report = dict(kind="position", position=position, speed=speed, status="RUNNING",
                  hardware=False, payload="retained", release_commands=0,
                  physics="stock ArduPilot X quad; concept CAD is visual only", events=[])
    try:
        world = json.loads((ROOT/"simulation/world.json").read_text())
        with SitlSession(directory, flight_enabled=True) as sitl:
            link = sitl.link
            coordinator = Coordinator(obstacles=[Obstacle(**item) for item in world["obstacles"]])
            service = CompanionService(link, coordinator)
            health = link.health()
            now = time.monotonic()
            request = {"position": list(position), "speed": speed}
            plan = service.request_navigation(coordinator.auth.sign(request, now),
                                              health, now, landed=link.landed, armed=link.armed)
            report["loaded_roundtrip_estimate_wh"] = plan.loaded_roundtrip_wh
            link.start_simulated_flight()
            coordinator.airborne()
            reached_target = False
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                link.recv()
                if any(event.get("mavpackettype") == "MISSION_ITEM_REACHED" and event.get("seq") == 3
                       for event in link.events) and not reached_target:
                    reached_target = True
                    coordinator.transition(State.RETURNING, "target waypoint reached; continuing home")
                if link.mode != "AUTO" and link.armed and coordinator.state in (State.OUTBOUND, State.RETURNING):
                    coordinator.transition(State.ABORTED, f"flight controller changed to {link.mode}")
                if coordinator.state in (State.OUTBOUND, State.RETURNING):
                    service.tick(link.health(), time.monotonic(), 40.)
                if link.landed and not link.armed:
                    break
            else:
                raise TimeoutError("navigation landing")
            if coordinator.state == State.ABORTED:
                raise Rejected("mission aborted during monitored flight")
            if not reached_target:
                raise Rejected("target waypoint was not confirmed")
            coordinator.finish(math.hypot(link.position.north, link.position.east),
                               link.landed, link.armed)
            report.update(status="PASS", final_state=coordinator.state.value,
                          final_position_neu=[link.position.north, link.position.east, link.position.up],
                          waypoint_reached=[e["seq"] for e in link.events
                                            if e.get("mavpackettype") == "MISSION_ITEM_REACHED"])
            return report
    except Exception as error:
        report.update(status="FAIL", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        if 'coordinator' in locals():
            report["events"] = coordinator.events
        (directory/"result.json").write_text(json.dumps(report, indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--north", required=True, type=float)
    parser.add_argument("--east", required=True, type=float)
    parser.add_argument("--up", required=True, type=float)
    parser.add_argument("--speed", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = run((args.north, args.east, args.up), args.speed, args.output)
    print(f"{result['status']}: {result['final_state']}; evidence: {args.output}/result.json")


if __name__ == "__main__":
    main()

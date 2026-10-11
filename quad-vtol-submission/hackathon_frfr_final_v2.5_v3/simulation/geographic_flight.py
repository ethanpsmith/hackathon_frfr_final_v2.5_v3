"""Full-distance WGS84 route in a new owned SITL child; no hardware connection."""
import argparse
import json
import math
from pathlib import Path
import time

from quadstack.core import Plan, Rejected, State
from quadstack.geographic import GeographicCoordinator, GeographicRoute, distance
from quadstack.service import CompanionService
from .transport import ROOT, SitlSession

CATALOG = json.loads((ROOT/"simulation/columbia-routes.json").read_text())


def resolve_request(body):
    if not isinstance(body, dict):
        raise Rejected("expected route request")
    if set(body) == {"route_id", "altitude", "speed"}:
        if not isinstance(body["route_id"], str) or body["route_id"] not in CATALOG:
            raise Rejected("unknown Columbia route")
        path = CATALOG[body["route_id"]]["path"]
    elif set(body) == {"origin", "target", "altitude", "speed"}:
        path = [body["origin"], body["target"]]
    else:
        raise Rejected("expected route_id or origin/target, plus altitude and speed")
    return GeographicRoute.parse(dict(path=path, altitude=body["altitude"], speed=body["speed"]))


def run(route, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    report = dict(kind="geographic", status="RUNNING", route=route.summary(), hardware=False,
                  payload="retained", release_commands=0, events=[])
    coordinator = GeographicCoordinator()
    def phase(name):
        (directory/"phase.json").write_text(json.dumps(dict(phase=name,payload="retained",
                                                        release_commands=0))+"\n")
    try:
        phase("PREPARING")
        with SitlSession(directory, flight_enabled=True, geographic_route=route) as sitl:
            link = sitl.link
            # Read back independent FC boundaries before mission upload/arming.
            for name, expected in {"ARMING_CHECK":1,"FENCE_ENABLE":1,"FENCE_TYPE":3,
                                   "FENCE_RADIUS":route.fence_radius,"FENCE_ALT_MAX":100,
                                   "FS_GCS_ENABLE":1,"FS_THR_ENABLE":1,"BATT_FS_LOW_ACT":2}.items():
                if abs(link.parameter(name)-expected) > .01:
                    raise Rejected(f"unexpected FC safety parameter {name}")
            health = link.health()
            now = time.monotonic()
            payload = dict(path=[list(p) for p in route.path], altitude=route.altitude, speed=route.speed)
            plan = coordinator.submit_geographic(coordinator.auth.sign(payload,now),health,now,
                         landed=link.landed, armed=link.armed, expected_route=route)
            service = CompanionService(link,coordinator)
            outbound = Plan(plan.points[:len(route.path)-1],plan.speed,plan.destination,plan.loaded_roundtrip_wh)
            returning = Plan(plan.points[len(route.path)-1:],plan.speed,plan.destination,plan.loaded_roundtrip_wh)

            def fly_leg(leg, name):
                # Both armings are normal, explicitly owned SITL operator actions.
                # The companion policy has no arming or motor-control interface.
                try:
                    link.upload(leg)
                except Exception:
                    coordinator.transition(State.ABORTED,f"{name} mission upload failed")
                    raise
                event_index = len(link.events)
                link.start_simulated_flight()
                if name == "outbound":
                    coordinator.airborne()
                    phase("OUTBOUND")
                else:
                    phase("RETURNING")
                reached = set()
                # Stock SITL runs at 5x; wall deadline includes landing and margin.
                deadline = time.monotonic()+(route.outbound_m/route.speed+route.altitude/.5)/5*2+60
                mission_seq = 3
                while time.monotonic()<deadline:
                    link.recv()
                    for event in link.events[event_index:]:
                        if event.get("mavpackettype") == "MISSION_ITEM_REACHED":
                            reached.add(event["seq"])
                    event_index = len(link.events)
                    if link.mode != "AUTO" and link.armed:
                        coordinator.transition(State.ABORTED,f"flight controller changed to {link.mode}")
                    current = link.messages.get("MISSION_CURRENT")
                    mission_seq = max(mission_seq,current.seq if current else 3)
                    index = max(0,min(len(leg.points)-1,mission_seq-3))
                    if name == "outbound":
                        return_m = route.outbound_m+route.altitude
                    else:
                        return_m = link.position.distance(leg.points[index])+route.altitude
                        return_m += sum(leg.points[i].distance(leg.points[i+1]) for i in range(index,len(leg.points)-1))
                    if not link.landed:
                        service.tick(link.health(),time.monotonic(),return_m)
                    if link.landed and not link.armed:
                        break
                else:
                    raise TimeoutError(f"geographic {name} landing")
                if coordinator.state == State.ABORTED:
                    raise Rejected(coordinator.events[-1]["reason"])
                expected=set(range(3,3+len(leg.points)))
                if not expected.issubset(reached):
                    raise Rejected(f"{name} incomplete; missing waypoint confirmations {sorted(expected-reached)}")
                return sorted(reached)

            outbound_reached=fly_leg(outbound,"outbound")
            target_error=distance(route.path[-1],route.global_position(link.position)[:2])
            if target_error>2 or not link.landed or link.armed:
                coordinator.transition(State.ABORTED,"target landing not established")
                raise Rejected("target landing not established")
            coordinator.transition(State.AT_PAD,"target landed and disarmed; geographic pad support unverified")
            phase("TARGET_LANDED")
            report["target_landing_error_m"]=target_error
            # Keep the landing observable to the dashboard before a new mission
            # is uploaded; continue receiving FC traffic and renew the lease.
            link.drain(3)
            # No surveyed compatible pad or support/latch sensing exists here.
            # A physical or simulated release is therefore never requested.
            coordinator.transition(State.DELIVERY,"simulated delivery stop; payload retained without support evidence")
            if coordinator.return_leg("retained",route.outbound_m+route.altitude,
                                      link.health(),time.monotonic()) is None:
                phase("HOLD")
                raise Rejected("return health or loaded reserve inadequate; remain at target")
            returning_reached=fly_leg(returning,"return")
            coordinator.finish(math.hypot(link.position.north,link.position.east),link.landed,link.armed)
            phase("COMPLETE")
            gps = link.messages["GLOBAL_POSITION_INT"]
            report.update(status="PASS",final_state=coordinator.state.value,
                          outbound_waypoint_reached=outbound_reached,
                          return_waypoint_reached=returning_reached,
                          waypoint_reached=outbound_reached+returning_reached,
                          final_gps=[gps.lat/1e7,gps.lon/1e7],
                          home_error_m=distance(route.path[0],(gps.lat/1e7,gps.lon/1e7)))
            return report
    except Exception as error:
        phase("FAILED")
        report.update(status="FAIL",error=f"{type(error).__name__}: {error}")
        raise
    finally:
        report["events"] = coordinator.events
        (directory/"result.json").write_text(json.dumps(report,indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", required=True, help="JSON geographic request")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = run(resolve_request(json.loads(args.request)),args.output)
    print(f"{result['status']}: {result['final_state']}")


if __name__ == "__main__":
    main()

"""Explicit simulation operator and test runner; no hardware target exists."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time
from quadstack.core import Coordinator, Health, Obstacle, State, Rejected
from quadstack.delivery import DeliveryState
from quadstack.sensors import delivery_scenario, DELIVERY_SCENARIOS
from quadstack.service import CompanionService
from .transport import SitlSession, ROOT

FAULTS = ("low_battery", "gnss_loss", "gnss_degradation", "lost_telemetry", "companion_crash", "rc_loss")


def await_landing(link, service):
    deadline = time.monotonic()+90
    while time.monotonic() < deadline:
        link.recv()
        c = service.coordinator
        if link.mode != "AUTO" and link.armed and c.state in (State.OUTBOUND,State.RETURNING):
            c.transition(State.ABORTED, f"FC/operator changed to {link.mode}; companion yields")
        if c.state in (State.OUTBOUND,State.RETURNING):
            service.tick(link.health(),time.monotonic(),40.)
        if link.landed and not link.armed:
            if c.state == State.ABORTED:
                raise Rejected("mission aborted during monitored flight; payload retained")
            return
    raise TimeoutError("monitored flight landing")


def make_service(link):
    world = json.loads((ROOT/"simulation/world.json").read_text())
    c = Coordinator(obstacles=[Obstacle(**o) for o in world["obstacles"]])
    return CompanionService(link,c), world["request"]


def run_scenario(name, directory, *, enable_release=False, flight=True):
    report = dict(scenario=name, status="RUNNING", hardware=False,
                  physics="stock ArduPilot quad; not compact BOM performance model", events=[])
    directory = Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    try:
        with SitlSession(directory,flight_enabled=flight) as sitl:
            link = sitl.link
            parameters = {n:link.parameter(n) for n in ("ARMING_CHECK","FENCE_ENABLE","FENCE_TYPE","FS_GCS_ENABLE","FS_OPTIONS","FS_THR_ENABLE","FS_EKF_ACTION","GRIP_ENABLE","FRAME_TYPE")}
            assert parameters == dict(ARMING_CHECK=1.,FENCE_ENABLE=1.,FENCE_TYPE=3.,FS_GCS_ENABLE=1.,FS_OPTIONS=0.,FS_THR_ENABLE=1.,FS_EKF_ACTION=1.,GRIP_ENABLE=0.,FRAME_TYPE=1.)
            report["verified_parameters"] = parameters
            service, request = make_service(link)
            c = service.coordinator
            # Authentication key exists only in this process and is never logged.
            now = time.monotonic()
            health = link.health()
            now = time.monotonic()
            plan = service.request(c.auth.sign(request,now),health,now,landed=link.landed,armed=link.armed)
            report["loaded_roundtrip_estimate_wh"] = plan.loaded_roundtrip_wh
            report["events"].extend(c.events)
            if not flight:
                report.update(status="PASS", mission="uploaded; disarmed; no flight requested")
                return report
            link.start_simulated_flight()
            c.airborne()
            if name in FAULTS:
                fault_start = len(link.events)
                if name == "low_battery":
                    link.parameter("SIM_BATT_VOLTAGE",10.9)
                elif name == "gnss_loss":
                    link.parameter("SIM_GPS_DISABLE",1)
                elif name == "gnss_degradation":
                    link.parameter("SIM_GPS_NUMSATS",3)
                    link.wait(lambda _: link.messages["GPS_RAW_INT"].satellites_visible<6,10,"reported GNSS degradation")
                    assert service.tick(link.health(),time.monotonic(),40.) == "LAND"
                    report["recovery_owner"] = "companion requests LAND; FC accepts"
                elif name == "rc_loss":
                    link.parameter("SIM_RC_FAIL",1)
                else:
                    sitl.stop_companion(crash=name=="companion_crash")
                expected = ("LAND",) if name.startswith("gnss") else ("RTL","LAND")
                link.wait(lambda _: link.mode in expected,45,f"independent FC {name} recovery")
                report["failsafe_mode"] = link.mode
                report["fault_telemetry"] = [e for e in link.events[fault_start:] if e.get("mavpackettype") in ("STATUSTEXT","HEARTBEAT")]
                if c.state != State.ABORTED:
                    c.transition(State.ABORTED,f"independent flight-controller {link.mode} after {name}")
                    report["recovery_owner"] = "flight controller independently"
                link.wait(lambda _: link.landed and not link.armed,75,"failsafe landing/disarming")
                report.update(payload="retained",release_commands=0,final_state=c.state.value)
            elif name in ("blocked_pad","camera_target_loss","rangefinder_failure"):
                link.wait(lambda _: math.hypot(link.position.north-20,link.position.east-10)<5 and link.position.up>5,
                          45,"approach staging point")
                mode=c.approach(link.health(),time.monotonic(),destination_valid=True,
                                pad_clear=name!="blocked_pad",target_valid=name!="camera_target_loss",
                                range_valid=name!="rangefinder_failure")
                assert mode=="RTL"
                report["approach_abort_altitude_m"]=link.position.up
                link.request_recovery(mode)
                link.wait(lambda _: link.landed and not link.armed,75,"loaded approach-abort return")
                assert math.hypot(link.position.north,link.position.east)<2
                from quadstack.core import energy_wh
                report.update(payload="retained",release_commands=0,final_state=c.state.value,
                              failsafe_mode=mode,recovery_owner="companion approach policy",
                              return_estimate_wh=energy_wh(40,plan.speed,True))
            else:
                assert c.approach(link.health(),time.monotonic(),destination_valid=True,pad_clear=True,
                                  target_valid=True,range_valid=True) is None
                await_landing(link,service)
                p = link.position
                c.at_pad(math.hypot(p.north-20,p.east-10),link.landed)
                reached = [e["seq"] for e in link.events if e.get("mavpackettype")=="MISSION_ITEM_REACHED"]
                assert 3 in reached and 4 in reached, f"waypoints not both confirmed: {reached}"
                c.begin_delivery()
                gp = link.messages["GLOBAL_POSITION_INT"]
                speed = math.hypot(gp.vx,gp.vy)/100.
                # Ground range is a mock 15 cm installation offset. Landed and speed
                # come from SITL telemetry; all cradle/fiducial evidence is synthetic.
                delivery = delivery_scenario(name,enable=enable_release,landed=link.landed,speed=speed)
                report["delivery_events"] = delivery.events
                report["payload"] = delivery.payload
                report["release_commands"] = getattr(delivery.actuator,"events",[]).count("release")
                report["waypoint_reached"] = reached
                budget = c.return_leg(delivery.payload,40.,link.health(),time.monotonic())
                report["return_estimate_wh"] = budget
                if budget is not None:
                    link.upload(plan,returning=True)
                    link.start_simulated_flight()
                    await_landing(link,service)
                    p = link.position
                    c.finish(math.hypot(p.north,p.east),link.landed,link.armed)
                else:
                    assert link.landed and not link.armed
                report["final_state"] = c.state.value
                report["final_position_neu"] = [link.position.north,link.position.east,link.position.up]
            report["events"] = c.events
            assert any(e["event"]=="mission_state" for e in c.events)
            report["status"] = "PASS"
    except Exception as e:
        report.update(status="FAIL",error=f"{type(e).__name__}: {e}")
        raise
    finally:
        if 'c' in locals():
            report["events"] = c.events
        (directory/"result.json").write_text(json.dumps(report,indent=2)+"\n")
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario",choices=(*DELIVERY_SCENARIOS,*FAULTS),default="nominal")
    p.add_argument("--sitl-flight",action="store_true",help="explicitly enable arming/flying ONLY the owned SITL child")
    p.add_argument("--enable-simulated-release",action="store_true",help="enable in-memory latch; no physical output")
    p.add_argument("--output",type=Path,default=ROOT/"simulation/runs/demo")
    args = p.parse_args()
    report = run_scenario(args.scenario,args.output,enable_release=args.enable_simulated_release,flight=args.sitl_flight)
    for event in report["events"]:
        print(f'{event.get("state",event["event"]):12s} {event.get("reason","")}')
    print(f'{report["status"]}: {report.get("final_state",report.get("mission"))}; payload={report.get("payload","retained")}')
    print(f"Evidence: {args.output}/result.json")


if __name__ == "__main__":
    main()

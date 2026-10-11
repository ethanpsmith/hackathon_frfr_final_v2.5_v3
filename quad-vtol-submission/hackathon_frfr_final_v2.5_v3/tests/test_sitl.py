import json
from pathlib import Path
import pytest
from simulation.demo import run_scenario
from simulation.transport import ROOT
from pymavlink import mavutil

SCENARIOS = ("nominal","release_disabled","low_battery","lost_telemetry","companion_crash",
             "gnss_loss","gnss_degradation","rc_loss","rangefinder_failure","camera_target_loss",
             "blocked_pad","false_contact","jammed_latch","failed_release")


@pytest.mark.sitl
@pytest.mark.parametrize("scenario",SCENARIOS)
def test_sitl_scenario(scenario):
    directory=ROOT/"simulation/runs/suite"/scenario
    report=run_scenario(scenario,directory,enable_release=scenario!="release_disabled")
    assert report["status"]=="PASS"
    assert (directory/"telemetry.jsonl").stat().st_size>0
    assert json.loads((directory/"manifest.json").read_text())["hardware"] is False
    outgoing=[json.loads(line) for line in (directory/"outgoing.jsonl").read_text().splitlines()]
    forbidden={mavutil.mavlink.MAV_CMD_DO_SET_SERVO,mavutil.mavlink.MAV_CMD_DO_GRIPPER,mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST}
    assert not any(e.get('command') in forbidden for e in outgoing)
    assert all(e.get('param2')==0 for e in outgoing if e.get('command')==mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    command=json.loads((directory/"manifest.json").read_text())["command"]
    assert not any('uart:' in x or '/dev/' in x or 'tcpclient:' in x for x in command)
    if scenario=="nominal":
        assert report["final_state"]=="COMPLETE"
        assert report["payload"]=="separated"
        assert report["release_commands"]==1
    elif scenario=="failed_release":
        assert report["final_state"]=="HOLD"
        assert report["payload"]=="unknown"
    elif "failsafe_mode" in report:
        assert report["final_state"]=="ABORTED"
        assert report["release_commands"]==0
    else:
        assert report["final_state"]=="COMPLETE"
        assert report["payload"]=="retained"


@pytest.mark.sitl
def test_disarmed_validation_and_all_parameter_readback():
    import time
    from quadstack.core import Rejected, State
    from simulation.transport import SitlSession
    from simulation.demo import make_service
    directory=ROOT/'simulation/runs/suite/disarmed_validation'
    with SitlSession(directory) as sitl:
        link=sitl.link
        expected={line.split()[0]:float(line.split()[1]) for line in (ROOT/'simulation/sitl.parm').read_text().splitlines() if line and not line.startswith('#')}
        actual={name:link.parameter(name) for name in expected}
        assert actual==pytest.approx(expected)
        service,request=make_service(link)
        c=service.coordinator
        cases=[dict(request,speed=9),dict(request,destination='unknown'),dict(request,waypoints=[[120,0,8],[20,10,8]])]
        for seq,req in enumerate(cases):
            now=time.monotonic()
            with pytest.raises(Rejected):
                service.request(c.auth.sign(req,now,seq),link.health(),time.monotonic(),landed=link.landed,armed=link.armed)
        now=time.monotonic()
        with pytest.raises(Rejected):
            service.request(c.auth.sign(request,now-6,10),link.health(),now,landed=True,armed=False)
        c.state=State.OUTBOUND
        with pytest.raises(Rejected):
            service.request(c.auth.sign(request,now,11),link.health(),time.monotonic(),landed=True,armed=False)
        c.state=State.IDLE
        with pytest.raises(Rejected):
            service.request(c.auth.sign(request,now,12),link.health(),time.monotonic(),landed=False,armed=False)
        before=[json.loads(x) for x in (directory/'outgoing.jsonl').read_text().splitlines()]
        assert not any(x['mavpackettype']=='MISSION_COUNT' for x in before)
        with pytest.raises(RuntimeError,match='explicit'):
            link.start_simulated_flight()
        now=time.monotonic()
        service.request(c.auth.sign(request,now,13),link.health(),time.monotonic(),landed=link.landed,armed=link.armed)
        assert not link.armed
        (directory/'validation.json').write_text(json.dumps(dict(status='PASS',readback=actual,rejections=c.events),indent=2))

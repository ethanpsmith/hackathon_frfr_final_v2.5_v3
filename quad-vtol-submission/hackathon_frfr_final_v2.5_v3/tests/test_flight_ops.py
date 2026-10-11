import json
import math
import pytest

from quadstack.core import Coordinator, Health, Obstacle, Rejected, State
from quadstack.service import CompanionService
from simulation.transport import ROOT


class CaptureController:
    def __init__(self):
        self.plans = []

    def upload(self, plan):
        self.plans.append(plan)


def request(payload, *, health=Health(10), landed=True, armed=False, obstacles=(), issued=10, now=10):
    controller = CaptureController()
    coordinator = Coordinator(obstacles=obstacles)
    service = CompanionService(controller, coordinator)
    envelope = coordinator.auth.sign(payload, issued)
    return service, controller, envelope, health, now, landed, armed


def test_position_request_uploads_bounded_round_trip():
    service, controller, envelope, health, now, landed, armed = request({"position":[20,10,8],"speed":3})
    plan = service.request_navigation(envelope, health, now, landed=landed, armed=armed)
    assert service.coordinator.state == State.VALIDATED
    assert [(p.north,p.east,p.up) for p in plan.points] == [(20,10,8),(0,0,8)]
    assert controller.plans == [plan]
    assert plan.destination == "position"


@pytest.mark.parametrize("payload",[
    {"position":[81,0,8],"speed":3}, {"position":[0,0,8],"speed":3},
    {"position":[20,10,2],"speed":3}, {"position":[20,10,16],"speed":3},
    {"position":[20,10,8],"speed":6}, {"position":[20,10,8],"speed":True},
    {"position":[20,10],"speed":3},
    {"position":[20,10,8],"speed":3,"release":True},
])
def test_invalid_position_never_uploads(payload):
    service, controller, envelope, health, now, landed, armed = request(payload)
    with pytest.raises(Rejected):
        service.request_navigation(envelope, health, now, landed=landed, armed=armed)
    assert controller.plans == []


def test_nonfinite_position_cannot_be_authenticated():
    with pytest.raises(Rejected, match="invalid JSON"):
        request({"position":[20,10,float('nan')],"speed":3})


def test_position_obstacle_and_health_and_freshness():
    for kwargs in (dict(obstacles=(Obstacle(10,5,2,8),)),
                   dict(health=Health(10,gnss_ok=False)), dict(issued=3),
                   dict(landed=False), dict(armed=True)):
        service, controller, envelope, health, now, landed, armed = request(
            {"position":[20,10,8],"speed":3}, **kwargs)
        with pytest.raises(Rejected):
            service.request_navigation(envelope,health,now,landed=landed,armed=armed)
        assert not controller.plans


@pytest.mark.sitl
def test_position_sitl_round_trip():
    from simulation.fly_to import run
    directory=ROOT/"simulation/runs/suite/position_navigation"
    report=run((20,10,8),3,directory)
    assert report["status"] == "PASS" and report["final_state"] == "COMPLETE"
    assert report["payload"] == "retained" and report["release_commands"] == 0
    assert 3 in report["waypoint_reached"]
    outgoing=[json.loads(line) for line in (directory/"outgoing.jsonl").read_text().splitlines()]
    assert not any(event.get("command") in (183,211,209) for event in outgoing)
    command=json.loads((directory/"manifest.json").read_text())["command"]
    assert not any("/dev/" in item or "uart:" in item or "tcpclient:" in item for item in command)

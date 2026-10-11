import json
import math
import pytest
from quadstack.core import Health, Rejected, State
from quadstack.geographic import GeographicCoordinator, GeographicRoute, distance
from simulation.geographic_flight import CATALOG, resolve_request
from simulation.transport import ROOT


def route(key="UAV-COLA-2047", **changes):
    return resolve_request(dict(route_id=key,altitude=43,speed=5,**changes))


def test_routes_preserve_full_distance_and_geodesy():
    expected={"UAV-COLA-2047":924.17,"UAV-COLA-3186":1448.63,"UAV-COLA-5729":1112.66}
    for key,metres in expected.items():
        r=route(key)
        assert r.outbound_m == pytest.approx(metres,abs=.01)
        for point in r.path:
            local=r.local(*point,43)
            lat,lon,alt=r.global_position(local)
            assert distance(point,(lat,lon)) < 1e-5 and alt==43
            assert math.hypot(local.north,local.east) < r.fence_radius-99
        assert r.path == tuple(tuple(p) for p in CATALOG[key]["path"])


@pytest.mark.parametrize("change",[
    {"altitude":19},{"altitude":81},{"speed":0},{"speed":6},{"speed":True},
    {"path":[[34,-81.04],[35,-81.04]]},{"path":[[34,-81.04],[34,-81.04]]},
    {"path":[[34,-81.04],[34.04,-81.04]]},{"path":[[34,-81.04],[float('nan'),-81.03]]},
    {"path":[[34,-81.04]]},{"release":True},
])
def test_geographic_invalid_inputs(change):
    body=dict(path=CATALOG["UAV-COLA-2047"]["path"],altitude=43,speed=5)
    body.update(change)
    with pytest.raises(Rejected):GeographicRoute.parse(body)


def test_geographic_authenticated_plan_and_reverse_route():
    r=route(); c=GeographicCoordinator();body=dict(path=[list(p) for p in r.path],altitude=43,speed=5)
    plan=c.submit_geographic(c.auth.sign(body,10),Health(10),10,landed=True,armed=False,expected_route=r)
    assert c.state==State.VALIDATED and len(plan.points)==2*(len(r.path)-1)
    for actual,expected in zip(plan.points,r.path[1:]+tuple(reversed(r.path[:-1]))):
        assert distance(r.global_position(actual)[:2],expected)<1e-5


@pytest.mark.parametrize("fault",["battery","gnss","stale_health","stale_request","armed","airborne","profile","replay"])
def test_geographic_policy_rejections(fault):
    r=route();c=GeographicCoordinator();body=dict(path=[list(p) for p in r.path],altitude=43,speed=5)
    health=Health(10,battery_wh=20) if fault=="battery" else Health(10,gnss_ok=False) if fault=="gnss" else Health(1) if fault=="stale_health" else Health(10)
    envelope=c.auth.sign(body,1 if fault=="stale_request" else 10)
    if fault=="replay":c.auth.verify(envelope,10)
    with pytest.raises(Rejected):c.submit_geographic(envelope,health,10,landed=fault!="airborne",armed=fault=="armed",expected_route=route("UAV-COLA-5729") if fault=="profile" else r)
    assert c.plan is None


@pytest.mark.parametrize("key",list(CATALOG))
@pytest.mark.sitl
def test_full_distance_columbia_sitl(key):
    from simulation.geographic_flight import run
    r=route(key);directory=ROOT/"simulation/runs/suite"/key
    report=run(r,directory)
    assert report["status"]=="PASS" and report["final_state"]=="COMPLETE"
    assert report["home_error_m"]<2 and report["target_landing_error_m"]<2
    assert report["payload"]=="retained" and report["release_commands"]==0
    assert [event["state"] for event in report["events"] if event["event"]=="mission_state"] == [
        "VALIDATED","OUTBOUND","AT_PAD","DELIVERY","RETURNING","COMPLETE"]
    manifest=json.loads((directory/"manifest.json").read_text())
    assert manifest["hardware"] is False and "--home" in manifest["command"]
    assert not any("/dev/" in s or "uart:" in s or "tcpclient:" in s for s in manifest["command"])
    outgoing=[json.loads(x) for x in (directory/"outgoing.jsonl").read_text().splitlines()]
    assert not any(x.get("command") in (183,211,209) for x in outgoing)
    uploads=[]
    for item in outgoing:
        if item.get("mavpackettype")=="MISSION_COUNT":uploads.append({})
        elif item.get("mavpackettype")=="MISSION_ITEM_INT":uploads[-1][item["seq"]]=item
    assert len(uploads)==2
    for uploaded,path in zip(uploads,(r.path[1:],tuple(reversed(r.path[:-1])))):
        assert uploaded[1]["command"]==22 and uploaded[2]["command"]==178
        assert uploaded[max(uploaded)]["command"]==21
        for seq,p in enumerate(path,3):
            assert distance(p,(uploaded[seq]["x"]/1e7,uploaded[seq]["y"]/1e7))<.02
    assert sum(x.get("command")==400 and x.get("param1")==1 for x in outgoing)==2

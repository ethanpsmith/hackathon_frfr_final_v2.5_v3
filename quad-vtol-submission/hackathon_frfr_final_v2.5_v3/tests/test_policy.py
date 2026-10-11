from dataclasses import replace
import copy
import math
import pytest
from quadstack.core import Authenticator, Coordinator, Health, Obstacle, Point, Rejected, State, energy_wh, to_gps, from_gps
from quadstack.delivery import Delivery, DeliveryState, Evidence, InertActuator
from quadstack.sensors import SensorModels, delivery_scenario, DELIVERY_SCENARIOS
from quadstack.service import CompanionService

REQUEST = {"destination":"delivery-pad","waypoints":[[10,0,8],[20,10,8]],"speed":3}


def accepted():
    c = Coordinator()
    c.submit(c.auth.sign(REQUEST,10),Health(10),10,landed=True,armed=False)
    return c


def test_roundtrip_states_and_coordinates():
    c = accepted()
    assert c.state == State.VALIDATED
    p = Point(20,10,8)
    back = from_gps(*to_gps(p))
    assert p.distance(back)<.00001
    c.airborne()
    c.at_pad(0.2,True)
    c.begin_delivery()
    loaded = c.return_leg("retained",40,Health(10),10)
    assert loaded == energy_wh(40,3,True)
    assert loaded > energy_wh(40,3,False)
    c.finish(.2,True,False)
    assert [e["state"] for e in c.events] == ["VALIDATED","OUTBOUND","AT_PAD","DELIVERY","RETURNING","COMPLETE"]


@pytest.mark.parametrize("mutation",[
    {"speed":float("nan")},{"speed":float("inf")},{"speed":True},{"speed":0},{"speed":6},
    {"waypoints":[]},{"waypoints":[[81,0,8],[20,10,8]]},{"waypoints":[[0,0,16],[20,10,8]]},
    {"waypoints":[[20,10,-1]]},{"waypoints":[[20,10]]},{"waypoints":[["20",10,8]]},
    {"waypoints":[[20,10,True]]},{"waypoints":[[0,0,8]]},{"destination":"unknown"},
    {"release":True},{"waypoints":[[20,10,8]]*21},
])
def test_malformed_route_never_forwards(mutation):
    class NeverSend:
        def upload(self,plan):
            pytest.fail("invalid route forwarded")
    service = CompanionService(NeverSend())
    req = copy.deepcopy(REQUEST)
    req.update(mutation)
    with pytest.raises(Rejected):
        service.request(service.coordinator.auth.sign(req,10),Health(10),10,landed=True,armed=False)
    assert service.coordinator.state == State.IDLE


@pytest.mark.parametrize("age",[-.1,5.01,100])
def test_freshness(age):
    c=Coordinator()
    with pytest.raises(Rejected):
        c.submit(c.auth.sign(REQUEST,10-age),Health(10),10,landed=True,armed=False)


def test_auth_tamper_replay_restart():
    a=Authenticator()
    e=a.sign(REQUEST,10)
    tampered=copy.deepcopy(e)
    tampered["body"]["payload"]["speed"]=4
    with pytest.raises(Rejected,match="unauthenticated"):
        a.verify(tampered,10)
    assert a.verify(e,10)==REQUEST
    with pytest.raises(Rejected,match="replayed"):
        a.verify(e,10)
    b=Authenticator(a.key)
    with pytest.raises(Rejected,match="session"):
        b.verify(e,10)


@pytest.mark.parametrize("landed,armed",[(False,False),(True,True),(False,True)])
def test_vehicle_state(landed,armed):
    c=Coordinator()
    with pytest.raises(Rejected):
        c.submit(c.auth.sign(REQUEST,10),Health(10),10,landed=landed,armed=armed)


def test_busy_and_bad_transitions():
    c=accepted()
    with pytest.raises(Rejected):
        c.submit(c.auth.sign(REQUEST,10,1),Health(10),10,landed=True,armed=False)
    with pytest.raises(Rejected):
        c.at_pad(0,True)
    with pytest.raises(Rejected):
        c.finish(0,True,False)
    c.airborne()
    with pytest.raises(Rejected):
        c.at_pad(2,True)


@pytest.mark.parametrize("health",[Health(0),Health(10,battery_wh=1),Health(10,voltage=21),Health(10,gnss_ok=False),Health(10,rc_ok=False),Health(10,telemetry_ok=False),Health(10,companion_ok=False),Health(10,estimator_ok=False)])
def test_go_no_go(health):
    c=Coordinator()
    with pytest.raises(Rejected):
        c.submit(c.auth.sign(REQUEST,10),health,10,landed=True,armed=False)


def test_segment_crossing_and_rtl_obstacles():
    # No waypoint lies in the first cylinder: the segment crosses it.
    for obstacle in (Obstacle(5,0,1,10),Obstacle(10,5,1,10),Obstacle(20,10,1,1)):
        c=Coordinator(obstacles=[obstacle])
        with pytest.raises(Rejected,match="blocked"):
            c.submit(c.auth.sign(REQUEST,10),Health(10),10,landed=True,armed=False)


@pytest.mark.parametrize("changes,expected",[
    ({"battery_wh":1},"RTL"),({"voltage":21.5},"RTL"),({"voltage":20.5},"LAND"),
    ({"gnss_ok":False},"LAND"),({"estimator_ok":False},"LAND"),({"timestamp":0},"LAND"),
    ({"telemetry_ok":False},"RTL"),({"companion_ok":False},"RTL"),({"rc_ok":False},"RTL"),
])
def test_health_faults(changes,expected):
    c=accepted()
    c.airborne()
    assert c.monitor(replace(Health(10),**changes),10)==expected
    assert c.state==State.ABORTED
    assert c.monitor(Health(11),11) is None  # recovery is latched; no automatic resumption


def test_inert_default_and_no_phase_authority():
    for d in (Delivery(0),Delivery(0,simulate_release=True)):
        assert isinstance(d.actuator,InertActuator)
        assert d.step(Evidence(0),0)==DeliveryState.RETAINED
    with pytest.raises(Rejected):
        InertActuator().release()


@pytest.mark.parametrize("field,value",[
    ("landed",False),("landed",1),("speed",.16),("altitude",.5),("range_m",0.09),
    ("range_valid",False),("destination_valid",False),("pad_clear",False),("target_visible",False),
    ("target_id",8),("target_confidence",.89),("contact",False),("supported_fraction",.89),
    ("latch_open",True),("timestamp",-1),("timestamp",1),("package_separated",True),
    ("speed",float("nan")),("range_m",float("inf")),("supported_fraction",True),
])
def test_each_interlock_prevents_release(field,value):
    d=Delivery(0,simulate_release=True,phase_authorized=True)
    d.step(replace(Evidence(0),**{field:value}),0)
    assert "release" not in d.actuator.events
    assert d.state in (DeliveryState.RETAINED,DeliveryState.UNCERTAIN)


def test_interlock_loss_during_dwell_and_after_opening():
    d=Delivery(0,simulate_release=True,phase_authorized=True)
    d.step(Evidence(0),0)
    d.step(Evidence(.5,contact=False),.5)
    assert "release" not in d.actuator.events
    d=Delivery(0,simulate_release=True,phase_authorized=True)
    d.step(Evidence(0),0)
    d.step(Evidence(1),1)
    assert d.state==DeliveryState.OPENING
    d.step(Evidence(1.1,pad_clear=False),1.1)
    assert d.state==DeliveryState.UNCERTAIN


@pytest.mark.parametrize("scenario",DELIVERY_SCENARIOS)
def test_delivery_fault_outcomes(scenario):
    d=delivery_scenario(scenario,enable=True)
    if scenario=="nominal":
        assert d.state==DeliveryState.RELEASED
        assert d.actuator.events==["release"]
    elif scenario in ("failed_release","contradictory_latch","stale_sensors"):
        assert d.state==DeliveryState.UNCERTAIN
    else:
        assert d.state==DeliveryState.RETAINED
    if scenario not in ("nominal","failed_release","jammed_latch"):
        assert "release" not in getattr(d.actuator,"events",[])


def test_unknown_payload_prevents_return():
    c=accepted()
    c.airborne()
    c.at_pad(0,True)
    assert c.return_leg("unknown",30,Health(10),10) is None
    assert c.state==State.HOLD


def test_seeded_noise_dropout_latency_range_and_battery_sag():
    a,b=SensorModels(seed=1),SensorModels(seed=1)
    assert a.rangefinder(1,10)==b.rangefinder(1,10)
    assert not a.rangefinder(13,10).valid
    assert not a.rangefinder(.05,10).valid
    assert a.rangefinder(1,10).timestamp<10
    assert not SensorModels(dropout=1).rangefinder(1,10).valid
    assert not a.gnss(0,0,10,lost=True).valid
    assert a.gnss(0,0,10,drift=20).value[0]>19
    assert not a.camera(10,visible=False).valid
    assert not a.compass(0,10,interference=True).valid
    assert a.battery(10,current=50).value["voltage"]<a.battery(10,current=1).value["voltage"]


def test_return_reserve_and_upload_failure():
    c=accepted()
    c.airborne()
    c.at_pad(0,True)
    assert c.return_leg("retained",40,Health(10,battery_wh=1),10) is None
    assert c.state==State.HOLD
    class Broken:
        def upload(self,plan):
            raise TimeoutError("no ACK")
    service=CompanionService(Broken())
    with pytest.raises(TimeoutError):
        service.request(service.coordinator.auth.sign(REQUEST,10),Health(10),10,landed=True,armed=False)
    assert service.coordinator.state==State.ABORTED


@pytest.mark.parametrize('field',['destination_valid','pad_clear','target_valid','range_valid'])
def test_approach_fault_retains_and_returns(field):
    c=accepted()
    c.airborne()
    conditions=dict(destination_valid=True,pad_clear=True,target_valid=True,range_valid=True)
    conditions[field]=False
    assert c.approach(Health(10),10,**conditions)=='RTL'
    assert c.state==State.ABORTED and c.payload=='retained'

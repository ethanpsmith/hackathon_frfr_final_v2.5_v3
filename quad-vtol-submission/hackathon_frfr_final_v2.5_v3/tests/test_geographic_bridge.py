"""HTTP route contract with mocked process launch; never starts a vehicle."""
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import pytest
from simulation import flight_ops_bridge as bridge


@pytest.fixture
def api(monkeypatch):
    captured=[]
    class Stub:
        token="test-only-bridge-code"
        directory=Path("mock-run")
        def launch(self,command,kind):captured.append((command,kind))
        def state(self):return dict(status="IDLE",telemetry={})
    monkeypatch.setattr(bridge,"supervisor",Stub())
    server=bridge.ThreadingHTTPServer(("127.0.0.1",0),bridge.Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    def send(body,token="test-only-bridge-code"):
        request=Request(f"http://127.0.0.1:{server.server_port}/api/geographic",
                        data=json.dumps(body).encode(),headers={"Content-Type":"application/json","X-Flight-Token":token})
        try:
            with urlopen(request) as response:return response.status,json.load(response)
        except HTTPError as response:return response.code,json.load(response)
    yield send,captured
    server.shutdown();server.server_close();thread.join()


@pytest.mark.parametrize("body",[
    dict(route_id="UAV-COLA-3186",altitude=43,speed=5),
    dict(origin=[33.998339,-81.044467],target=[34.006317,-81.041680],altitude=43,speed=5),
])
def test_route_api_launches_only_fixed_simulator_module(api,body):
    send,captured=api
    assert send(body)[0]==202
    command,kind=captured[0]
    assert command[:3]==["-m","simulation.geographic_flight","--request"]
    assert json.loads(command[3])==body and kind=="geographic"


@pytest.mark.parametrize("body,token,status",[
    (dict(route_id="UAV-COLA-3186",altitude=43,speed=5),"wrong",401),
    (dict(route_id="UAV-COLA-3186",altitude=43,speed=.5),"test-only-bridge-code",400),
    (dict(route_id="unknown",altitude=43,speed=5),"test-only-bridge-code",400),
    (dict(route_id="UAV-COLA-3186",altitude=43,speed=5,serial="/dev/ttyUSB0"),"test-only-bridge-code",400),
    (dict(origin=[34,-81.04],target=[34.04,-81.04],altitude=43,speed=5),"test-only-bridge-code",400),
    (dict(route_id="UAV-COLA-3186",altitude=float('nan'),speed=5),"test-only-bridge-code",400),
])
def test_route_api_rejects_before_any_process(api,body,token,status):
    send,captured=api
    assert send(body,token)[0]==status
    assert not captured


@pytest.mark.sitl
def test_custom_route_supervisor_streams_real_sitl():
    import time
    import os
    import signal
    supervisor=bridge.Supervisor()
    body=dict(origin=[33.998339,-81.044467],target=[33.99905,-81.04415],altitude=20,speed=5)
    supervisor.launch(["-m","simulation.geographic_flight","--request",json.dumps(body)],"geographic")
    seen=set();has_route=False;seen_phases=set()
    try:
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            state=supervisor.state()
            has_route |= bool(state.get("route"))
            if state.get("phase"):seen_phases.add(state["phase"]["phase"])
            gps=state["telemetry"].get("GLOBAL_POSITION_INT")
            if gps and gps.get("lat"):seen.add((gps["lat"],gps["lon"]))
            if state["status"]!="RUNNING":break
            time.sleep(.2)
        assert state["status"]=="PASS",state
        assert has_route and len(seen)>5
        assert {'OUTBOUND','TARGET_LANDED','RETURNING','COMPLETE'} <= seen_phases
        assert state["result"]["hardware"] is False
        assert state["result"]["home_error_m"]<2
        assert state["result"]["target_landing_error_m"]<2
        assert state["result"]["payload"]=="retained" and state["result"]["release_commands"]==0
    finally:
        if supervisor.process.poll() is None:
            os.killpg(supervisor.process.pid,signal.SIGTERM)
            supervisor.process.wait(10)
        supervisor.log.close()

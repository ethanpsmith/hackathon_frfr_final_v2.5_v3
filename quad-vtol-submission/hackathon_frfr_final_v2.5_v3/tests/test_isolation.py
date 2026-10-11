from pathlib import Path
from types import SimpleNamespace
import ast
import pytest
from pymavlink import mavutil
from quadstack.delivery import SimulatedActuator
from simulation.transport import Link, ROOT, SitlSession


def test_no_external_endpoint_or_hardware_target():
    with pytest.raises(TypeError):
        SitlSession('unused', endpoint='/dev/ttyUSB0')
    with pytest.raises(TypeError):
        SitlSession('unused', target='hardware')


def test_command_and_parameter_boundaries():
    link=Link(SimpleNamespace(mav=SimpleNamespace(set_send_callback=lambda _:None)),SimpleNamespace(flight_enabled=False))
    for command in (mavutil.mavlink.MAV_CMD_DO_SET_SERVO,mavutil.mavlink.MAV_CMD_DO_GRIPPER,mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST):
        with pytest.raises(ValueError):
            link.command(command,1)
    with pytest.raises(ValueError):
        link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
    link.session.flight_enabled=True
    with pytest.raises(ValueError):
        link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1,21196)
    for name in ('ARMING_CHECK','FENCE_ENABLE','FS_GCS_ENABLE','SERVO9_FUNCTION'):
        with pytest.raises(ValueError):
            link.parameter(name,0)


def test_payload_and_core_have_no_io_drivers():
    # Architectural regression test: flight policy cannot import transport/device libs.
    prohibited={'serial','socket','RPi','gpiozero','pymavlink','subprocess'}
    for path in (ROOT/'flight-software/quadstack').glob('*.py'):
        tree=ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Import):
                assert not any(x.name.split('.')[0] in prohibited for x in node.names)
            elif isinstance(node,ast.ImportFrom):
                assert (node.module or '').split('.')[0] not in prohibited
    actuator=SimulatedActuator()
    actuator.release()
    actuator.retain()
    assert actuator.__dict__=={'events':['release','retain']}


def test_sitl_parameters_preserve_failsafes_and_disable_gripper():
    values={line.split()[0]:float(line.split()[1]) for line in (ROOT/'simulation/sitl.parm').read_text().splitlines() if line and not line.startswith('#')}
    for key in ('ARMING_CHECK','FENCE_ENABLE','FS_GCS_ENABLE','FS_THR_ENABLE','FS_EKF_ACTION'):
        assert values[key]==1
    assert values['FS_OPTIONS']==0
    assert values['GRIP_ENABLE']==0

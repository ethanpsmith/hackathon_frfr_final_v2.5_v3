# Setup and operation

Run all commands at the repository root. `simulation/setup.sh` installs tools locally into ignored `.tools/` and `.venv/`. It does not flash firmware, open a serial device, install system services, change shell configuration or contact a printer/vehicle.

## Prerequisites

- Git and a native C/C++ compiler. Tested: Apple Clang 21.0.0, arm64 macOS 26.3.1/Darwin 25.3.0. `xcode-select -p` must locate command-line tools. If absent, manually install with `xcode-select --install`.
- [uv installation](https://docs.astral.sh/uv/getting-started/installation/). The script finds `uv` on PATH or under `~/.local/bin`.
- Network access for Python wheels and the ArduPilot source/submodules; several GB free for build and logs.
- Linux candidate setup: Ubuntu with `git build-essential` and uv, then the same script. **Not tested here.** Windows requires an appropriately configured Linux environment; no Windows-native claim.

```bash
bash simulation/setup.sh
.venv/bin/python -m pytest tests/test_policy.py tests/test_perception.py tests/test_isolation.py -q
bash simulation/demo.sh
bash simulation/demo.sh --sitl-flight
bash simulation/test.sh
```

The default demo uploads an accepted mission but does not arm. Only `--sitl-flight` and the explicitly invoked SITL test suite enable normal arming of the child simulator. The simulator must pass its pre-arm checks. There are no forced-arm magic values or disabled-failsafe shortcuts. `--enable-simulated-release` enables an in-memory actuator only; it cannot issue MAVLink gripper/servo messages.

## Exact pins and native build

| Component | Pin |
|---|---|
| CPython | 3.11.13 |
| ArduCopter | Copter-4.6.3, `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` |
| MAVLink source in firmware | submodule `bb87bc7390af7f21d9ad33a45c8be02997fecd24` |
| pymavlink client | 2.4.49, ardupilotmega dialect |
| MAVProxy | 1.8.74, installed as optional upstream development tool |
| OpenCV image-only detector | opencv-python-headless 4.12.0.88 |
| NumPy | 2.2.6 |
| Test runner | pytest 8.4.2 |
| Build generators | empy 3.3.4, setuptools 80.9.0 |
| All transitive packages | `simulation/requirements.lock` |

ArduPilot submodules are checked out to the commits recorded by the parent revision. `requirements.in` describes direct development requirements; the lock records the installed complete set. The native C++ build uses the host compiler and SDK and is not bit-for-bit portable. MAVProxy GUI dependencies are not installed; the headless demo connects using pymavlink and does not launch MAVProxy.

Build commands actually used, inside `.tools/ardupilot`:

```bash
../../.venv/bin/python waf configure --board sitl
../../.venv/bin/python waf copter -j 6
```

The harness launches `build/sitl/bin/arducopter` directly with:

```text
-S --model x --speedup 5 --home -35.363261,149.165230,584,0
--defaults <upstream>/Tools/autotest/default_params/copter.parm,<repo>/simulation/sitl.parm
--wipe
--serial0 udpclient:127.0.0.1:<owned-observer-port>
--serial1 udpclient:127.0.0.1:<owned-companion-port>
--serial2 none --serial3 GPS1 --serial4 GPS2
--serial5 none --serial6 none --serial7 none --serial8 none
--rc-in-port <fresh-test-port>
```

Ports are allocated per run; no endpoint is accepted from the operator. The selected native binary and parameter overlay are hashed into the per-run manifest. `--wipe` resets only the fresh run's SITL EEPROM. Upstream virtual sensor calibration defaults are used; none are physical calibration values. The adapter validates the FC source IDs and the expected firmware checkout.

ArduPilot's [SITL overview](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) and [supported launch interfaces](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html) describe the native simulator. The exact tested launcher is the one above, not an untested Gazebo/ROS configuration.

## Scenarios and results

```bash
bash simulation/demo.sh --sitl-flight --scenario low_battery
bash simulation/demo.sh --sitl-flight --scenario companion_crash
bash simulation/demo.sh --sitl-flight --scenario blocked_pad --enable-simulated-release
bash simulation/demo.sh --sitl-flight --scenario failed_release --enable-simulated-release
bash simulation/test.sh -k nominal
```

`failed_release` intentionally ends **HOLD**, landed and disarmed at the pad, because an open latch without positive separation does not establish a safe return configuration. That is a passing safety outcome, not a completed delivery.

Each run writes `result.json`, `manifest.json`, `telemetry.jsonl`, `sitl.log`, and upstream `logs/*.BIN`. Reusing an output directory replaces the JSON evidence for that run; use `--output simulation/runs/<unique-name>` to retain separate demonstrations. Ctrl-C tears down owned processes. No unrelated running simulator is killed.

A failed test exits nonzero and includes recent FC status messages. Do not resolve arming failures by disabling checks. Confirm GPS/estimator readiness and the stock-model parameter overlay first. The implementation distinguishes missing tools from passing checks.

Reproduction has been exercised in this working environment. **A second clean environment has not independently reproduced the setup.** The script and pins provide the recipe; they are not evidence of that unperformed check.

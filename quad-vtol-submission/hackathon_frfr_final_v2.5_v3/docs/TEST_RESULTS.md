# Verification results — 2026-10-10

## Target landing, return climb and dashboard diagnostics

The geographic SITL runner now executes two normal ArduCopter missions per route:
outbound navigation and LAND at the target, followed by a separately uploaded
TAKEOFF/reverse-route/LAND at home. The target coordinate and landed/disarmed
state are verified before the return; the package remains retained because
compatible-pad support and release evidence are unavailable. The owned simulator
made two ordinary arming requests and sent no gripper/servo release command.

```bash
PYTHONPATH=flight-software .venv/bin/python -m pytest tests/test_geographic.py::test_full_distance_columbia_sitl --sitl -q
PYTHONPATH=flight-software .venv/bin/python -m pytest tests/test_geographic.py tests/test_geographic_bridge.py --sitl -q -k 'not full_distance_columbia_sitl'
PYTHONPATH=flight-software .venv/bin/python -m pytest tests -q
node site/flight-ops/check.mjs
node site/flight-ops/check-runtime.mjs
```

The three full-distance route tests passed in **397.70 s**. Target-landing errors
were 0.024 m (2047), 0.022 m (3186) and 0.022 m (5729); each returned home and
disarmed with the payload retained. The targeted geographic/bridge suite passed
**30 tests in 43.92 s**, including the observed target-stop status during an owned
custom-route SITL run. The non-SITL suite passed **132 tests**, with 20 SITL tests
explicitly skipped. The 2047 replay was rebuilt from its new passing SITL run;
the static checks cover the target descent, landed stop, second climb, six changing
graphs, diagnostics conversion and separation from a prior completed bridge run.
Chrome preview showed updating replay telemetry and heading values with no console
errors. The full-distance runs preceded a later three-second dashboard-visible
target dwell; the targeted bridge SITL run exercised that dwell. These are stock
SITL results, not evidence of a physical payload drop-off or airworthiness.

## Flightpath UI restyle check

This UI-only update reused the existing bridge and flight software. The Flightpath
layout, five telemetry graph mappings and recorded replay were checked with:

```bash
node site/flight-ops/check.mjs
node site/flight-ops/check-runtime.mjs
PYTHONPATH=flight-software .venv/bin/python -m pytest tests/test_geographic_bridge.py --sitl -q
```

The static checks passed with 8 JavaScript modules and 189 recorded flight frames;
the bridge check passed **9 tests in 32.19 seconds**, including a custom GPS SITL
round trip with changing telemetry. Safari preview showed all five graphs, the
Flightpath street map and the STEP-derived drone. Selecting a different route
cleared the Finlay Park replay and its graph values. The 143-test full suite below
was run for the preceding flight-software revision and was not rerun for this
UI-only update.

**152 distinct checks passed, 0 failed, 0 errors, 0 skipped**, across these runs:

- Full suite: **143 passed in 652.58 seconds** (10:52).
- Added geographic bridge checks: **9 passed in 31.97 seconds**, including a custom-GPS round trip and changing live telemetry through the process supervisor.

```bash
bash simulation/test.sh
PYTHONPATH=flight-software .venv/bin/python -m pytest tests/test_geographic_bridge.py --sitl -q --junitxml=simulation/runs/geographic-bridge-junit.xml
```

This invokes:

```bash
PYTHONPATH="$PWD/flight-software" .venv/bin/python -m pytest --sitl tests -q --junitxml=simulation/runs/junit.xml
```

Breakdown: **132 unit/component/HTTP checks, 14 ArduCopter delivery/fault flight scenarios, 1 bounded position round trip, 3 full-distance Columbia round trips, 1 custom-GPS supervisor round trip and 1 disarmed live-SITL validation test**. The latter read back all **37** original overlay parameters and rejected malformed, stale, out-of-fence and inappropriate-state requests before mission upload. Valid upload stayed disarmed. Outgoing traffic checks found no gripper/servo/motor-test command or forced-arming value, and the launch commands contained no physical serial path. The 9 bridge tests were added after the full suite began collecting, so their separate XML result is preserved rather than claiming a single 152-test run.

## Columbia geographic integration

All three routes flew their original WGS84 waypoints at 43 m above assumed home,
5 m/s, with a 5× simulator clock. The full geographic waypoint sequence and reverse
return were confirmed by MAVLink. Payload remained retained; final state was
COMPLETE, landed and disarmed.

| Route | Planned outbound | Planned round trip | Final GPS distance from home |
|---|---:|---:|---:|
| Publix → Finlay Park | 924.169 m | 1,848.339 m | 0.024 m |
| Mast General → Riverfront Park | 1,448.628 m | 2,897.257 m | 0.014 m |
| Publix → Museum of Art | 1,112.662 m | 2,225.324 m | 0.022 m |

These are stock-SITL outputs, not physical navigation accuracy measurements.
The first exploratory long run exposed mission-index reset during disarm;
monotonic progress tracking fixed the erroneous post-landing return-budget check.
All three final integration runs passed without changing safety freshness checks
or disabling FC failsafes. See [geographic profile](GEOGRAPHIC_FLIGHTS.md).

| Flight scenario | Observed passing result |
|---|---|
| nominal, mock release explicitly enabled | Waypoints reached, pad landing, simulated release/separation, return home, disarm, COMPLETE |
| release_disabled | Same route with no release and loaded-return estimate, COMPLETE |
| low_battery | Independent FC RTL, landing and disarm |
| lost_telemetry | Independent FC RTL after designated heartbeat loss, landing/disarm |
| companion_crash | Independent FC RTL after heartbeat-process SIGKILL, landing/disarm |
| gnss_loss | Independent FC LAND and disarm |
| gnss_degradation | Reported satellite count fell below threshold; companion requested LAND; FC landed/disarmed |
| rc_loss | Independent FC RTL, landing/disarm |
| rangefinder_failure | Approach aborted at 8.004 m; retained payload, RTL home and disarm |
| camera_target_loss | Approach aborted at 8.005 m; retained payload, RTL home and disarm |
| blocked_pad | Approach aborted at 8.004 m; retained payload, RTL home and disarm |
| false_contact | No release, confirmed retained payload, loaded return home |
| jammed_latch | Mock release attempt only after valid support; closed latch timed out, retention verified, loaded return home |
| failed_release | Open latch without separation confirmation: uncertain payload, HOLD, no second takeoff |
| position_navigation | Requested local `(20, 10, 8)` m, reached target, returned home, landed/disarmed, COMPLETE; payload retained, no release command |

The full scenario summaries, firmware/binary/parameter hashes and tested Python source hashes are preserved in [test-evidence.json](test-evidence.json). Large raw evidence is local and ignored by Git: `simulation/runs/junit.xml`, `simulation/runs/suite/*/result.json`, `telemetry.jsonl`, `outgoing.jsonl`, `manifest.json`, `sitl.log` and DataFlash `logs/*.BIN`. Raw files can contain local paths; the reusable evidence summary does not.

Additional commands executed during initial setup and subsequent dashboard work
(the setup/operator-demo commands belong to the earlier 119-check revision):

```bash
bash simulation/setup.sh
bash simulation/demo.sh --sitl-flight --output simulation/runs/operator-demo
.venv/bin/mavproxy.py --version
.venv/bin/python -m compileall -q flight-software simulation tests
PYTHONPATH=flight-software .venv/bin/python -m pytest tests/test_flight_ops.py --sitl -q
node site/flight-ops/check.mjs
node site/flight-ops/check-runtime.mjs
git diff --check
```

The setup script completed successfully after the earlier 119-check run, synchronizing pinned dependencies and rebuilding native SITL without changing the binary hash. That revision's operator demo then passed again with release disabled: COMPLETE, payload retained, home landing and disarm. MAVProxy reported **1.8.74** (with an upstream pkg_resources deprecation warning); macOS `gnureadline==8.2.13` and `setuptools==80.9.0` are pinned. The geographic update added `geographiclib==2.1` using `~/.local/bin/uv pip install --python .venv/bin/python geographiclib==2.1`. Compilation, asset/import checks, UI contract checks and whitespace checks passed again for this update. Setup was exercised on macOS 26.3.1 arm64, Apple Clang 21.0.0, Python 3.11.13. A different clean machine, Linux and a physical Pi were **not** tested.

Native Safari review confirmed route cards, full-distance values, route selection
and the combined layout. The console showed no runtime errors during that review.
Live bridge behavior was verified by the HTTP and supervisor integration tests;
an interactive browser-driven live launch and a narrow mobile viewport were not
exercised in this update. The existing user bridge was not stopped or replaced.

Earlier integration failures were corrected before the passing suite: missing generator dependency; simulator GPS selector syntax; estimator readiness delay; stock-model battery mismatch; and the incorrect expectation that satellite-count reduction alone triggers an independent EKF failsafe. During Flight Ops development, two full runs encountered a safe companion LAND request after a transient heartbeat freshness gap under heavy browser/logging load. The final passing run used sampled telemetry file writes and the reviewed Site limits animation work. The one-second health threshold, arming checks, fence and failsafes were not disabled.

## Limits of this evidence

- Flight execution used the stock **3 kg / 350 mm / 3S** ArduPilot X model, not the selected **5.70 kg / 650 mm / 6S** concept.
- Cradle, contact/load, range and camera scenario inputs remain synthetic. OpenCV detection was tested on generated marker images, not camera hardware.
- No motors, ESCs, GPIOs, servos or physical releases were accessed. A simulated release is an in-memory event.
- The static obstacle map is a route validation aid; no collision physics, moving-obstacle avoidance or validated pad occupancy detector was exercised.
- The conceptual energy estimates are not fitted propulsion/battery models. Payload release does not change stock simulator mass.
- Geographic routes use inherited unsurveyed presentation coordinates, assumed 100 m MSL flat ground, and no city buildings/terrain. Distances are full scale; city route clearance and selected-aircraft endurance are unverified.
- Companion crash tests kill the designated heartbeat process while a separate test observer remains. They do not characterize every Pi/OS failure or an RF radio link.
- Hardware interfaces, independent release interlocks, actual 6S thresholds/calibration, Pi performance and Remote ID installation remain to be verified under the [bench prerequisites](BENCH.md).

These passes establish only the tested software behavior and assumptions, not aircraft flight readiness or regulatory approval.

# Models, coverage and interpretation

The flight backend is the pinned upstream X quad, with stock dynamics and native GNSS/IMU/barometer/compass/battery/RC simulation. It is **not** a validated MN4110/P16/FLAME/650 mm dynamics model. Stock defaults include 3 kg, 0.35 m diagonal, 12.6 V maximum and unlimited physical battery capacity; the FC capacity counter still integrates simulated current. No payload mass change is applied to these dynamics.

`world.json` defines level pads, waypoint and obstacle geometry. The mission runs in local north/east/up coordinates translated into GPS mission items. No Gazebo, terrain, moving-object physics or collision engine is present. Planned paths and emergency-return corridors are checked against known cylinders; unknown obstacles are not detected.

## Synthetic selected-device observations

`SensorModels` uses a seeded RNG (650 by default) and timestamps measurements at capture time minus configured latency. Default numeric range noise σ=0.015 m, latency 0.05 s and dropout probability 0.0; tests exercise probability 1 and explicit invalidation. These choices are stress-test assumptions, not manufacturer accuracy specifications.

| Observation | Model and fault knobs | What is not established |
|---|---|---|
| M10-like GNSS | NE noise σ=0.3 m, additive drift, loss flag; native SITL GPS can be disabled or report fewer satellites | Actual M10 performance, multipath or antenna layout |
| Compass | Heading noise σ=1°, interference invalidation | Installed magnetic environment/calibration |
| TFmini-like range | 0.1–12 m nominal clipping, noise, dropout, latency, explicit failure | Reliable outdoor range; return quality on actual surfaces |
| Camera | Marker ID, visibility, quality/offset observations; target-loss fault | Pad occupancy or safe descent inferred from a marker |
| Battery | 6S OCV proxy `6*(3.3+0.9*SOC)`, configurable current and resistance sag; assumed 10 Ah | Validated cell model, usable capacity or flight range |
| RC / telemetry | Boolean availability samples; independent native FC fault tests | RF link-budget, interference, receiver wiring |
| Contact / load / latch | Explicit digital states, supported load fraction, separation; contradiction/staleness injection | Sensor independence or mechanical qualification |

The delivery scenario's ground range is a **synthetic 0.15 m sensor mounting height**. Landed state and horizontal speed come from actual SITL telemetry; all pad, contact/load, latch and separation evidence is synthetic. Simulation time in these deterministic payload tests is an injected clock, separate from real-time FC integration. No sensor fault result should be described as a physical-device test.

Optional image-only perception uses OpenCV's ArUco `DICT_4X4_50`, marker 7 and caller-provided intrinsics. It returns angular centre and apparent size, preserving capture timestamps. It rejects >640×480 frames, stale images, invalid intrinsics and absent/ambiguous markers. Tests generate 320×240 marker images and perform real detection. [OpenCV's detector documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) describes the API. No camera capture, metric pose, FC LANDING_TARGET command, pad-clear classifier or Pi frame-rate claim is implemented. Camera support is advisory and cannot authorize release alone.

## Executed scenario contracts

| Scenario | Injection / layer | Required outcome |
|---|---|---|
| nominal | ArduCopter outbound/return; mock supported release enabled | Both waypoint reached events, pad landing, release + separation, home landing/disarm, COMPLETE |
| release_disabled | Same real SITL route; mock actuation off | No release, loaded return estimate, COMPLETE |
| low_battery | Native `SIM_BATT_VOLTAGE=10.9` in stock 3S model | Independent FC RTL, landing/disarm; no release |
| lost_telemetry | Stop designated heartbeat process cleanly | Independent FC RTL, landing/disarm; observer continues logging |
| companion_crash | SIGKILL companion health process | Same independent FC recovery |
| gnss_loss | Native `SIM_GPS_DISABLE=1` | Independent FC LAND and disarm |
| gnss_degradation | Native `SIM_GPS_NUMSATS=3`; observe <6 satellites | Companion policy requests LAND, FC accepts, lands/disarms |
| rc_loss | Native `SIM_RC_FAIL=1` | Independent FC recovery/landing/disarm |
| rangefinder_failure | Invalid synthetic range at >5 m approach staging altitude | Companion RTL before touchdown; retained loaded return |
| camera_target_loss | Invalid synthetic marker observation during approach | Companion RTL before touchdown; retained loaded return |
| blocked_pad | Synthetic pad-clear false during approach | Companion RTL before touchdown; retained loaded return |
| false_contact | Contact true, load transfer zero | No release; retained loaded return |
| jammed_latch | Mock release permitted after support; latch remains closed | Confirmation timeout; positively retained payload; loaded return |
| failed_release | Mock latch opens, separation remains false | UNCERTAIN, HOLD at pad, no return takeoff |
| disarmed_validation | Live SITL with all overlay parameter readbacks, malformed/stale/outside-fence/active-state requests | Reject before any MISSION_COUNT; valid upload remains disarmed |

Additional unit scenarios include missing pad, absent weight transfer, contradictory latch signals, stale/future/nonfinite data, explicit abort, phase timeout, loss of interlocks during dwell/after opening, authentication/signature/session/replay checks, segment crossings, insufficient return reserve, wrong-state transitions, upload failure, target image rejection and command/parameter isolation.

The low-satellite-count stock simulation continued AUTO without an independent EKF failsafe in the initial experiment. The final degradation test therefore asserts **companion policy recovery**, explicitly distinct from native GNSS-loss recovery. It would be incorrect to claim that the FC independently lands on satellite count alone.

Unknown destinations are rejected before dispatch. Blocked-pad, lost-camera and invalid-range scenarios abort at an approach staging point above 5 m, within 5 m horizontally of the destination, and request RTL with the payload retained. Known-clear evidence in nominal runs is synthetic. This exercises approach policy, not real occupancy detection or precision landing. Unit tests separately verify that the same faults at the ground-supported delivery phase inhibit release.

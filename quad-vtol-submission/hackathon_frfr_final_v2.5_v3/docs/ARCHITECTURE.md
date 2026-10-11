# Architecture and trust boundaries

`quadstack.core` is a pure state machine, authentication and route/energy policy. `quadstack.service.CompanionService` accepts authenticated mission requests and calls a narrow `FlightController` protocol: upload a validated plan or request RTL/LAND. It has no arm, parameter-write, motor, servo or GPIO interface. `quadstack.delivery` implements a separately gated payload phase. `quadstack.perception` consumes an already captured grayscale image, not a camera device. `quadstack.sensors` supplies explicitly synthetic observations.

The runnable transport is in `simulation/transport.py` and is intentionally outside the installable companion package. There is no hardware driver or hardware-mode switch. The simulator operator, not the companion API, explicitly arms and starts each simulated leg after the FC's checks. ArduCopter owns attitude/rate control, estimator, mission execution and failsafes throughout.

## Mission request contract

A local client receives an ephemeral 32-byte session key through trusted process setup. The demo creates and uses it in memory. A request consists of `body` plus a SHA-256 HMAC `mac`. Body fields are exactly `session`, monotonically increasing integer `sequence`, monotonic `issued`, and `payload`. The payload is exactly:

```json
{"destination":"delivery-pad","waypoints":[[10,0,8],[20,10,8]],"speed":3}
```

Canonical JSON sorts keys, removes insignificant whitespace and forbids NaN/Infinity. HMAC comparison is constant-time. Wrong signatures/sessions, repeated sequence numbers, future timestamps and age >5 seconds are rejected. Authenticated rejected requests consume their sequence. Restart creates a new session; the API cannot resume a previously airborne mission. This is a local request protocol, not network identity provisioning or a hardware MAVLink signing installation.

Policy rejects unknown/extra fields, unknown pads, a destination-coordinate mismatch, nonnumeric/nonfinite values, bools used as numbers, speed outside 0.5–5 m/s, routes outside an 80 m radius or 3–15 m transit altitude, more than 20 waypoints, unhealthy/stale telemetry, insufficient estimated loaded-roundtrip reserve, and any state other than idle/landed/disarmed. Static obstacles are checked along segments, takeoff/descent and direct emergency-return corridors. Terrain and moving obstacles are not modeled.

Mission states: `IDLE → VALIDATED → OUTBOUND → AT_PAD → DELIVERY → RETURNING → COMPLETE`. `ABORTED` latches a health fault; `HOLD` prevents return takeoff with uncertain payload/latch status. Pad arrival requires landed state and <=1.5 m position error; completion requires home distance <=2 m, landed and disarmed. Actual `MISSION_ITEM_REACHED` events establish waypoint completion. No automatic in-flight retry after a failsafe.

## Coordinates and energy

Local axes are north/east/up. The origin is `(-35.363261°,149.165230°)`, ground AMSL 584 m. Mapping uses Earth radius 6,378,137 m and longitude scaled by cos(origin latitude). MAVLink uses GLOBAL_RELATIVE_ALT_INT: degrees×1e7 and metres above launch ground. The approximation is restricted to this <=100 m demo and level pads; it is not a general geodesic/terrain planner.

The separate concept policy assumes 900 W loaded / 650 W empty, transit time = route distance/speed, 30 s contingency and 20% reserve. These are deliberately labeled sizing proxies, not motor data or the prior trade study's range calculation. Outbound approval budgets a loaded return. A failed delivery with confirmed closed latch uses loaded energy; unknown latch/payload holds at the pad. The stock simulator's 3S voltage is doubled only in an explicit concept-policy adapter; stock FC battery failsafes use unscaled 3S thresholds. Percentage is mapped to the assumed 222 Wh nominal concept pack. Neither is calibrated state-of-energy estimation.

## MAVLink ownership and timeouts

| Owner | Messages / behavior |
|---|---|
| FC, system 1/component 1 | HEARTBEAT, SYS_STATUS, GPS_RAW_INT, GLOBAL_POSITION_INT, EXTENDED_SYS_STATE, mission reached/current, COMMAND_ACK, PARAM_VALUE, STATUSTEXT |
| Local SITL operator, system 254/component 190 | Stream requests, readback, mission upload handshake, explicit normal simulator arming/start, allowlisted SIM_* fault injection |
| Companion health process, system 255/component 191 | Sole SYSID_MYGCS heartbeat source; five-times-clock demo sends at 10 Hz wall time |
| Companion policy | Validates requests before upload; can request only RTL/LAND recovery through its narrow adapter |
| Payload | In-memory release/retain events; **no MAV_CMD_DO_GRIPPER/DO_SET_SERVO** |

Upload responds to MISSION_REQUEST/INT and requires accepted MISSION_ACK plus all item requests within 20 wall seconds. Command ACK timeout is 5 s; requested modes must be observed within 8 s. Parameter reads retry three times. Normal takeoff deadline is 20 s; each flight-leg landing deadline 90 s. These are wall-clock integration-test deadlines, not real-vehicle timing settings.

The health process must receive FC traffic and a renewed supervisor lease within 2 wall seconds. A dead/hung supervisor cannot leave indefinite heartbeats running. FC GCS loss timeout is 5 simulated seconds; `FS_OPTIONS=0` does not permit AUTO to bypass it. Tests stop the heartbeat process cleanly or kill it with SIGKILL, while a separate observer continues collecting evidence. This exercises loss of the designated companion heartbeat, not RF propagation or every possible operating-system crash.

Core health policy requests LAND on stale/invalid position/estimator or critical voltage, RTL on link/RC/companion loss, low voltage or insufficient return reserve, subject to FC acceptance. Independent FC battery, RC, GCS and complete-GNSS-loss recovery is tested without companion recovery commands so its ownership is observable. Satellite-count degradation separately exercises companion-requested LAND. Blocked pad, camera loss and invalid range exercise companion-requested RTL before touchdown.

## Payload interlocks

No remote release verb exists. Local code can enter delivery only after verified pad arrival. Release is inert unless both the explicit simulated-actuator opt-in and delivery-phase authority are present. A one-second stable dwell requires all of:

- fresh evidence (<=0.5 s, never future), landed, speed <=0.15 m/s, altitude <=0.3 m;
- valid range 0.1–0.3 m, consistent with relative mounting/altitude assumption;
- registered destination, pad-clear confirmation, visible marker 7 and sufficient synthetic observation quality;
- positive contact and >=90% independent load-transfer estimate;
- contradictory latch signals absent, latch closed, package still captive.

After the mock release, fresh open-latch **and** separation evidence are required. Missing support/stale evidence aborts immediately; latch/separation confirmation times out after 2 s; total phase times out after 8 s. A closed jammed latch is retained. An open latch with failed separation, contradictory latch signals or stale state is uncertain and prevents return. A retain request is not proof of re-latching. No free-drop phase exists.

## Isolation limits

The launcher owns its local child and UDP receivers; it has no user-selectable serial or external network endpoint. Its writable parameter and command APIs are allowlisted. The core imports no hardware I/O libraries, and no actuator message is emitted. Tests check these boundaries and actual launch/traffic evidence. This is application-level isolation, **not an OS sandbox against hostile local code**. The upstream SITL process has auxiliary simulator sockets; it is not a hardened network service. Use a trusted local development machine, not an exposed multi-user flight-control server.

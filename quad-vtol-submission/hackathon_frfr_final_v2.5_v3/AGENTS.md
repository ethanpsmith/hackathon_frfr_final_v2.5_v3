# Agent instructions: Quad delivery VTOL

## Project purpose and source of truth

This project describes a compact, electrically powered quad VTOL concept and supports development of its simulation and flight-software prototype. Treat the CAD model and study assumptions as concept inputs, not evidence of airworthiness. Read `README.md` before changing the design or adding flight-software behavior. Preserve the distinction between the compact short-route study configuration and the larger 5-mile candidate described in the trade study; do not mix their dimensions or power systems.

The current model is the 650 mm diagonal wheelbase, four-rotor, 16-inch-propeller configuration. Its study takeoff mass is approximately 5.70 kg with a 1.814 kg (4 lb) package. The model itself does not calculate mass, center of gravity, range, endurance, thrust margin, structural strength, or flightworthiness. Keep those quantities labeled as assumptions until independently calculated and verified.

## Selected hardware constraints

Use these as the current candidate BOM when writing interfaces, configuration, or simulations. Verify exact part revisions, electrical limits, connector pinouts, firmware support, and sensor interfaces against manufacturer documentation before encoding them as facts.

- Flight controller: Holybro Pixhawk 6C. It is the primary flight computer and owns ArduCopter attitude/rate control, motor outputs, estimator inputs, core failsafes, and flight modes.
- Companion computer: Raspberry Pi Zero 2 W. Treat it as a constrained companion for high-level mission coordination and lightweight perception only. Do not place stabilization, motor mixing, or safety-critical immediate control loops on it.
- Candidate propulsion: four T-Motor MN4110 KV400 motors, four FLAME 60 A ESCs, 16-inch propellers, and a 6S 10 Ah Tattu battery. Confirm the exact propeller model, ESC firmware/protocol, current limits, and propulsion data before performance claims.
- Power and sensing: PM08 CAN power module/PDB, M10 GNSS/compass, TFmini Plus downward rangefinder (nominal stated range 0.1–12 m), and Camera Module 3 standard. Do not assume the rangefinder is valid for all surfaces, heights, lighting, or approach conditions.
- Communications/identity: RP4TD RC receiver, SiK telemetry radio, and Holybro Remote ID candidate. Simulated links must not be mistaken for a compliant installed Remote ID system.
- Payload release: custom captive cradle/latch with servo actuation and latch/contact or load sensing. The concept sequence is pad-supported delivery: establish package support and verify weight transfer before release. No free-drop behavior is part of the concept.

If a source document, manufacturer specification, or validated measurement conflicts with this list, record the discrepancy and use the better-supported value. Do not silently alter the BOM or CAD assumptions.

## Flight-software architecture

- Build around the supported ArduCopter firmware for the Pixhawk 6C; do not replace it with a custom autopilot.
- Keep the flight controller authoritative for stabilization, motor control, state estimation, arming checks, mode transitions, geofence, battery failsafe, RC failsafe, and loss-of-companion/link behavior.
- Use the companion for bounded high-level tasks over a documented MAVLink interface: mission requests, health/status reporting, and optional visual target estimation. Define message ownership, timeouts, validity checks, and fallback behavior explicitly.
- Treat all companion-computer requests as untrusted inputs. Validate coordinates, altitude, speed, mission state, geofence, vehicle state, and command freshness before forwarding any request. A companion disconnect or crash must not disable the flight controller's independent failsafes.
- Implement delivery as an explicit state machine. Require landed/low-motion state, compatible pad confirmation, positive package support/weight-transfer evidence, valid latch state, and deliberate test enablement before simulated actuation. On contradictory or stale sensor data, retain the payload and abort/retry safely.
- Keep hardware I/O behind interfaces so SITL and unit tests use mocks. Do not invent Pixhawk GPIO assignments, servo outputs, CAN IDs, serial ports, or sensor drivers; derive them from the verified wiring plan and board configuration.

## Simulation and development workflow

ArduPilot SITL and MAVProxy are external development tools, not repository plugins. Use ArduCopter SITL as the first test target. Start with the ArduPilot-provided multicopter model and document the exact firmware revision, simulator backend, parameter file, and commands that were actually tested. Do not claim reproducibility until another clean environment can follow the recorded setup.

Gazebo Harmonic with `ardupilot_gazebo` is an optional visual simulation layer for camera, landing-pad, and obstacle scenarios. Keep the first software-in-the-loop path runnable without Gazebo. ROS 2 is optional and should be added only when a concrete multi-node perception or sensor-integration need justifies its setup burden.

Keep reusable project artifacts in this directory. Put software and simulation code in clearly named subdirectories (for example `flight-software/`, `simulation/`, and `tests/`) rather than mixing it into CAD source files. Add a setup guide and a version/parameter manifest when introducing the toolchain. Pin dependencies where feasible and avoid committing generated build outputs, credentials, radio keys, or local machine paths.

## Safety and test limits

- Default all tests to SITL, mocked sensors, and mocked actuators. Never connect a test script to a physical vehicle by default.
- The payload servo/release output must be inert in SITL unless a test explicitly opts into a simulated actuator. Never energize a real payload release during software development or simulation setup.
- Never configure tests to bypass arming checks, geofence, failsafes, or hardware interlocks as a shortcut. If a narrowly scoped SITL-only override is necessary, isolate it in a clearly named test configuration and prove it cannot be loaded by a hardware build.
- Do not auto-arm, spin motors, launch a vehicle, or actuate hardware. Hardware-in-the-loop or physical testing requires an explicit, reviewed test procedure and a controlled test environment.
- Report simulation success only for the scenarios and model assumptions actually tested. SITL success does not establish flight readiness, range, release reliability, regulatory compliance, or safety.

## Required verification for changes

For every software or simulator change, run the relevant unit tests and SITL tests and report the exact commands and results. At minimum, the planned suite should cover:

1. Valid waypoint mission request and completion in SITL.
2. Rejection of malformed, stale, out-of-geofence, or inappropriate-state mission requests.
3. Companion disconnect/crash, telemetry loss, low-battery, GNSS degradation/loss, and rangefinder invalid-data scenarios.
4. Delivery state-machine tests for missing pad, absent weight transfer, stuck/contradictory latch sensor, timeout, and abort; all must retain the payload unless release preconditions are satisfied.
5. Confirmation that a normal SITL run cannot reach a physical serial port, real servo, ESC, or payload actuator.

If a test cannot run because an external tool is not installed, state that plainly and provide the reproducible setup needed. Do not label an unrun check as passed.

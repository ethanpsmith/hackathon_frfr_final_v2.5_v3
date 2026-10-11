# Parameters and selected hardware review

Reviewed against upstream source at Copter-4.6.3 (`92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`) and linked manufacturer/upstream documentation on 2026-10-10. The **only loadable overlay is SITL-only** `simulation/sitl.parm`. This document is an interface review, not a wiring plan or a flashable Pixhawk parameter set. No physical output, UART allocation, sensor CAN node ID or PWM endpoint has been assigned.

## Every SITL overlay parameter

The overlay follows the pinned upstream `Tools/autotest/default_params/copter.parm`. That file supplies stock-model PID tuning, virtual accelerometer/compass calibration, RC ranges, motor thrust curve and flight-mode defaults. These must never be copied as physical calibrations. Exact inherited defaults are preserved by the firmware commit.

| Parameter | Value | Meaning and scope |
|---|---:|---|
| FRAME_CLASS | 1 | Quad |
| FRAME_TYPE | 1 | X mixing; simulator must also use model `x` |
| ARMING_CHECK | 1 | All applicable arming checks |
| FENCE_ENABLE | 1 | Fence active |
| FENCE_TYPE | 3 | Maximum-altitude + circular fence |
| FENCE_RADIUS | 100 | metres; companion plans within 80 m |
| FENCE_ALT_MAX | 20 | metres relative altitude; planned transit <=15 m |
| FENCE_MARGIN | 2 | metres |
| FENCE_ACTION | 1 | RTL, with FC fallback as required |
| FS_THR_ENABLE | 1 | RC loss recovery enabled, RTL |
| FS_GCS_ENABLE | 1 | GCS loss always RTL, LAND if position unavailable |
| FS_GCS_TIMEOUT | 5 | simulated seconds without designated GCS heartbeat |
| FS_OPTIONS | 0 | No continue-AUTO exceptions |
| FS_EKF_ACTION | 1 | LAND for EKF failsafe |
| SYSID_MYGCS | 255 | Dedicated companion heartbeat owner in this test |
| BATT_MONITOR | 4 | Stock simulated analog voltage/current; **not PM08 CAN** |
| BATT_CAPACITY | 10000 | mAh capacity counter for test; stock dynamics battery has unlimited capacity |
| BATT_LOW_VOLT | 11.1 | **Stock 3S** low-voltage trigger; not physical 6S threshold |
| BATT_CRT_VOLT | 10.5 | **Stock 3S** critical trigger |
| BATT_LOW_MAH | 2000 | remaining capacity trigger |
| BATT_CRT_MAH | 1000 | critical remaining capacity trigger |
| BATT_FS_LOW_ACT | 2 | Copter RTL |
| BATT_FS_CRT_ACT | 1 | Copter LAND |
| BATT_LOW_TIMER | 3 | seconds below threshold before battery failsafe |
| SIM_BATT_VOLTAGE | 12.6 | Matches stock dynamics 3S full voltage |
| MOT_BAT_VOLT_MIN | 9.6 | Stock compensation lower bound |
| MOT_BAT_VOLT_MAX | 12.8 | Stock compensation upper bound |
| WPNAV_SPEED | 300 | cm/s default; accepted mission speed is also encoded with DO_CHANGE_SPEED |
| WPNAV_ACCEL | 100 | cm/s² |
| WPNAV_RADIUS | 100 | cm arrival radius |
| RTL_ALT | 1000 | cm above home |
| RTL_LOIT_TIME | 1000 | ms before RTL descent |
| LAND_SPEED | 50 | cm/s final descent |
| DISARM_DELAY | 3 | seconds after landing |
| GRIP_ENABLE | 0 | FC gripper disabled; mock actuator has no MAVLink path |
| SERIAL1_PROTOCOL | 2 | MAVLink2-capable virtual companion port |
| LOG_DISARMED | 1 | Capture preflight diagnostic logs |

The live test reads critical values back and fails on mismatches. All overlay names/values are also read back by the disarmed integration test. Parameters and units are sourced from the pinned `ArduCopter/Parameters.cpp`, `libraries/AC_Fence`, `AP_BattMonitor`, `AP_Motors`, `AP_SerialManager` and SITL source. [Current parameter reference](https://ardupilot.org/copter/docs/parameters.html) can differ from the pin; source wins for this executable.

## Selected airframe and propulsion

650 mm diagonal implies motor centres at ±229.81 mm in forward/right body axes. Propeller diameter is 406.4 mm. Adjacent centre spacing is 459.62 mm and static geometric tip gap is about 53.2 mm. These geometric numbers do not account for bending, tolerances or dynamic propeller clearance.

ArduPilot X ordering viewed from above, nose forward:

| Motor function | Body position (forward, right), mm | Spin |
|---|---|---|
| Motor1 | (+229.8,+229.8), front right | CCW |
| Motor2 | (-229.8,-229.8), rear left | CCW |
| Motor3 | (+229.8,-229.8), front left | CW |
| Motor4 | (-229.8,+229.8), rear right | CW |

Verified in pinned `AP_MotorsMatrix::setup_quad_matrix` and [motor-order documentation](https://ardupilot.org/copter/docs/connect-escs-and-motors.html). These are **functions**, not a confirmed physical connector assignment. Motor-test ordering differs from motor numbering.

[T-Motor's MN4110 page](https://store.tmotor.com/product/mn4110-motor-navigator-type.html) lists the KV400/P16×5.4/6S combination and manufacturer test data; the [FLAME 60A 12S V2 page](https://store.tmotor.com/product/flame-60a-12s-V2-esc.html) identifies the ESC candidate. Those records do not validate this integrated airframe. No DShot/ESC protocol, PWM endpoints, thrust table, motor current limits or thermal margins are encoded. Verify the shipped SKU/firmware, signal input specification, BEC arrangement, actual current and thrust on a protected test stand. The 12S rating in the ESC name does not make this a 12S power system. Tattu 6S/10 Ah/30C remains a selected pack description; exact SKU, wiring and measured capacity are unresolved.

## Pixhawk 6C and electrical interfaces

[ArduPilot Pixhawk 6C support](https://ardupilot.org/copter/docs/common-holybro-pixhawk6C.html) identifies the `Pixhawk6C` firmware target and UART mapping: SERIAL1/TELEM1, SERIAL2/TELEM2, SERIAL3/GPS1, SERIAL4/GPS2, SERIAL5/TELEM3 on full-size 6C. The same page's RC example has inconsistent UART naming; use the pinned board hwdef plus Holybro pinout before wiring. The physical board revision, cable pinouts, supply rails and available UART budget have **not** been verified on an actual board.

- **PM08 CAN:** [Holybro DroneCAN setup](https://docs.holybro.com/power-module-and-pdb/power-module/dronecan-power-module-setup) specifies CAN1, `CAN_P1_DRIVER=1`, `BATT_MONITOR=8`, and documents `CAN_SLCAN_CPORT=1`, `BATT_OPTIONS=1`. Confirm DroneCAN protocol, termination, node discovery, battery-ID selection, scaling, capacity semantics and shipped firmware. The 6C's analog power-port example (`BATT_MONITOR=4`) is a different interface; it must not be substituted for PM08 CAN. The requested “300 A PDB” is a candidate rating, not an approved continuous system limit.
- **M10 GNSS/compass:** [Holybro M10](https://holybro.com/products/m10-gps) offers compass variants. Confirm the actual IST8310/BMM150 choice, connector, UART GNSS and I²C compass wiring, sensor orientation, offsets and interference. Do not hard-code compass device IDs or calibration. Default simulated GNSS/compass is not this device's accuracy model.
- **TFmini Plus:** [ArduPilot UART/I²C integration](https://ardupilot.org/copter/docs/common-benewake-tfmini-lidar.html) distinguishes serial type 20 from I²C type 25. For a verified UART allocation, protocol 9 and baud enum 115 mean 115200 baud. Downward orientation enum is 25. **Version discrepancy:** current docs use `RNGFND1_MIN/MAX` in metres; pinned 4.6.3 source uses `RNGFND1_MIN_CM/MAX_CM` in centimetres. Nominal 0.1–12 m is not a reliable outdoor limit; upstream recommends a 6 m outdoor maximum. Mount height, surface reflectivity/sunlight behavior and real interface mode remain unverified. No UART or I²C address is assigned by this repository.
- **RP4TD:** [RadioMaster](https://www.radiomasterrc.com/products/rp4td-expresslrs-2-4ghz-diversity-receiver) lists CRSF. On 6C this requires a true UART, not the timer-based RCIN connector. Candidate serial protocol 23/options 0 applies only after port/electrical review. Receiver bind/firmware, loss behavior, mode-switch mapping and manual recovery must be bench tested. RC remains FC-owned.
- **SiK/companion telemetry:** MAVLink-capable UARTs must be allocated without conflict with GNSS/rangefinder/CRSF. Confirm radio baud/firmware, voltage levels, signed-link policy and bandwidth. The demo's system-255 heartbeat ownership must be redesigned/reviewed if a ground station shares or masks that identity. No RF behavior is modeled.
- **Pi Zero 2 W / Camera Module 3 standard:** [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html) covers the CSI connection and modern camera stack. Use an appropriate Zero camera cable and calibrate the installed camera. Image-only ArUco detection is tested at 320×240; no camera capture driver, calibration file, Pi OS image or Pi latency/thermal measurement is supplied. Capture and perception must never own immediate stabilization.
- **Holybro Remote ID:** [ArduPilot Remote ID](https://ardupilot.org/copter/docs/common-remoteid.html) describes supported interfaces. Exact module revision, serial/CAN interface, region, identity provisioning and firmware support are unresolved. This native build reports OpenDroneID disabled; it does not emulate or establish a compliant installation.
- **HS65MG captive latch:** [ArduPilot gripper documentation](https://ardupilot.org/copter/docs/common-gripper-servo.html) and pinned code identify servo gripper type 1 and output function 28. `GRIP_ENABLE` stays 0 here. Do not assign an output, PWM endpoints or GPIO sensing until the rail voltage/current, mechanical travel, retention load, stall behavior and independent interlock are verified. Contact/load electronics, calibration and pin assignments remain TBD. The servo's name is not evidence that the custom latch can safely retain this load.

There is intentionally no “hardware-ready” parameter file. Creating one requires a verified board/connector inventory, reserved output/UART groups and separate reviewed bench configuration. Loading the stock simulator calibration or battery settings onto the selected aircraft would be incorrect.

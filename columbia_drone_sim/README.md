# Columbia Autonomous Drone Delivery Simulator

A local Python desktop simulator for your **quad_delivery_vtol.stl**, a warehouse-to–Site A mission, landed package release, and return to the warehouse. Designed for VS Code. Includes the original STL, source code, editable parameters, offline Columbia weather references, optional NWS observations, tests, and example results.

**Simulation only. The STL provides geometry—not aircraft mass, aerodynamics, endurance, airworthiness, or flight authorization. All aircraft-performance parameters are explicit, unvalidated engineering assumptions. This code has no connection to real aircraft.**

## 1. Start in VS Code (Windows)

1. Extract the ZIP completely. Open the extracted `columbia_drone_sim` folder in VS Code. Do not run inside the ZIP.
2. Install Python 3.11 or 3.12 with its Tcl/Tk component, and the Microsoft Python extension for VS Code.
3. Open **Terminal → New Terminal** in that folder. Run:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py
```

Using the venv Python directly avoids PowerShell activation-policy problems. In VS Code, also select `.venv` using **Python: Select Interpreter**. The included F5 launch configurations run the nominal and gust examples.

macOS/Linux equivalent:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python main.py
```

On a Linux desktop without Tk, install your distribution's Python Tk package. On a server without a desktop, use `--headless`. An interactive window cannot appear in a headless terminal.

## 2. What appears

- **Mission corridor:** intended route and actual flown path, warehouse, Site A, and a virtual boundary. Horizontal and vertical scales differ so altitude changes are visible.
- **Your STL in 3D:** attitude changes, wind streaks, force-direction arrows, and an orange symbolic package. Drag a 3D view to rotate it. The original STL is preserved; a reduced display mesh speeds rendering.
- **Telemetry:** altitude, ground/air speed, wind at the drone, roll/pitch, SOC, voltage, power, temperatures, air density, payload, and estimated return energy.
- **Autonomy explanation:** current mission decision and the reason for a return, rejection, or emergency response.
- **Response plots:** altitude, cross-track error, and remaining battery against simulation time.

Buttons inject crosswind, headwind, rain, a battery fault, or an operator return. **Space** pauses; **G** injects gusts; **R** returns; **S** saves. Playback is adjustable from 1–32×; final descent, landed dwell, release, and departure confirmation automatically run at **1×**. Playback rate never changes the physics time step. Slow computers may run slower than the requested playback speed.

This is a technical desktop visualization, not a photorealistic game engine or a rendered map of Columbia.

## 3. Default mission and assumptions

| Item | Configured reference value | Status |
|---|---:|---|
| One-way route | 3 miles / 4,828.032 m | Virtual route, not approved geography |
| Cruise altitude | 60 m AGL | Project target below configured 121.92 m ceiling |
| Cruise groundspeed | 12 m/s | Unvalidated performance target |
| Maximum airspeed command | 18 m/s | Unvalidated project limit |
| Payload | 1.81437 kg / 4 lb | Assumed packaged payload |
| Aircraft mass including battery, excluding package | 5.8 kg | Assumption; must be weighed |
| Nominal loaded mass | 7.61437 kg | Sum of assumed aircraft and payload |
| Battery | 6S, 30 Ah, 666 Wh nominal | Assumed pack, not a selected product |
| Battery mass | 3.4 kg, already included in 5.8 kg | Assumed; do not add it twice |
| Battery state of health | 95% | Scenario assumption |
| Reserve | 30% of temperature-adjusted nominal capacity | Planning policy, not an FAA universal reserve |
| Final descent | 0.3 m/s | Slow project target |
| Landed dwell / release / confirmation | 10 / 3 / 6 seconds | Visible motor-safe release sequence |
| Physics step | 0.02 s / 50 Hz | Independent of screen frame rate |

The battery's implied pack energy density is about 196 Wh/kg. That is only arithmetic on assumptions; procurement, discharge capability, packaging, thermal safety, and flight suitability are **not established**. A larger battery increases mass: always update both fields and inertia together. The code does not automatically resize a real airframe or select hardware.

The mission first tests normal delivery **and a loaded return if delivery is denied**, including reserve. Changing a distance or load can produce `NO_GO`; the simulator does not force success. Try the 5-mile case to see this constraint.

## 4. Run weather and fault scenarios

In the commands below, `python` means the selected virtual environment interpreter. If your terminal has not activated it, use `.\.venv\Scripts\python.exe` instead.

```bash
python main.py --scenario gust
python main.py --scenario headwind
python main.py --scenario rain
python main.py --scenario battery
python main.py --scenario motor
python main.py --scenario motor_failure
python main.py --scenario lost_link
python main.py --scenario navigation_loss
python main.py --scenario blocked_pad
python main.py --scenario hot
python main.py --scenario fog
python main.py --distance-miles 5
python main.py --month 7
```

Most scheduled disturbances start at 150 simulated seconds. `gust` adds a northward wind step and larger turbulence, then clears at 195 s. `headwind` adds westward wind until 220 s. `motor` reduces one motor's maximum available thrust by 20%; this can remove reserve thrust without changing cruise behaviour if the existing demand still fits. A total motor failure is a different and much more serious test. The ordinary quadrotor controller is not a validated motor-out controller.

`battery` is an artificial sudden change to 20% charge; it is not a battery-aging model. `navigation_loss` marks navigation unavailable but uses an idealized local stabilization assumption to examine mission logic. It is **not** proof of GNSS-denied landing capability. Unsafe ground contact is reported as `CRASHED`, never a successful emergency landing.

## 5. Columbia weather: averages vs observations

The offline mode deliberately works without internet. For October it uses the NWS 1991–2020 monthly mean of **64.6°F / 18.1°C**. The annual temperature normal is **64.3°F / 17.9°C**. For reference wind and humidity, it uses the **2024 annual report**, which reports **5.8 mph / 2.59 m/s** mean wind and **70%** average relative humidity. These are different averaging periods, clearly labelled in `data/columbia_climate.json` and `WEATHER_AND_REGULATIONS.md`.

This combination is a **constructed climate scenario**, not a single observed day or a statistically joint weather model. Wind direction, turbulence, cloud distance, pressure, visibility, and daylight are editable assumptions. Monthly precipitation is context only; it is not spread evenly across every simulated flight. Rain, fog, hot weather and faults have separate tests. Surface wind is extrapolated with a simple assumed shear profile, not a local atmospheric survey.

To request a timestamped NWS station observation:

```bash
python main.py --weather live
```

Set a descriptive contact address in `weather.user_agent` first. The loader requests KCAE observations and active alerts, validates units and freshness, and saves `outputs/weather_snapshot.json`. It also reports recent sample averages using at most one report per UTC hour. Missing hours are excluded and sample counts are shown. These recent averages are **not** climate normals.

The newest valid snapshot initializes the flight. It is **not continuously refreshed** during the simulation; simulated gusts and events then evolve around it. No forecast, radar, onboard anemometer, or route-specific microclimate feed is included. Station observations do not measure conditions across the route or between buildings.

The live NWS request returned HTTP 403 in the development environment. The parser, unit handling, stale-data rejection and missing-data checks were tested with controlled fixtures; successful live retrieval here is **not claimed**. Live mode exits on retrieval/validation failure. Use `--weather climate` explicitly if you want the offline scenario—there is no silent fallback.

## 6. Package loading, delivery and return

The simulated warehouse worker loads the package and confirms the destination. There is only one order/site in this prototype. The mission then climbs, cruises, starts a descending approach within 450 m of Site A, aligns above the pad, and descends slowly. After ground contact, motors must decay to near zero. It waits, releases the cargo, updates mass/inertia, confirms separation, climbs again and returns to the hub.

No winch is used. The orange package is a separate symbolic marker; your STL remains a rigid assembly because STL does not specify articulated joints or a release mechanism. Package-bay geometry, a latch, sensors and reliable separation require actual mechanical design and testing.

Blocked-pad input represents the result of a future landing-zone sensor or operator check. This prototype does not run computer vision or detect real people. The emergency ground is an explicitly assumed clear flat surface, not a real surveyed alternate. Grounded safety holds require recovery rather than allowing takeoff through an occupied pad.

## 7. Change your aircraft or mission

Edit `config.json`. All distances are metres, masses kilograms, forces newtons, time seconds, and angles degrees unless named otherwise. The STL's assumed millimetre conversion is `mesh.units_to_m = 0.001`; scale and orientation are not encoded by STL itself. The model bounds become approximately 0.866 × 0.866 × 0.468 m at that scale.

Before considering results representative of your aircraft, replace:

1. Measured empty and loaded mass, component mass breakdown, centre of gravity and inertia.
2. Measured motor/propeller thrust and electrical-power maps versus RPM, voltage and density; ESC limits and time constants.
3. Pack discharge/capacity/internal-resistance/temperature data, current limits, BMS behaviour and mass.
4. Drag areas and moments, propeller interference, vibration and structural limits.
5. Verified sensor/control performance, aircraft weather limits and real landing mechanism data.
6. Surveyed warehouse/Site A coordinates, terrain, landing areas and operator-approved routes/contingencies.

`warehouse_lat_lon` is only a Columbia reference. The local route is in east/north metres; changing these latitude/longitude fields does not download terrain or grant airspace access. Match `weather.latitude/longitude` separately if changing the weather-alert location. No automated geocoding is performed.

## 8. Logs, testing and files

Each run saves into `outputs/<timestamp>_<scenario>/` when you press Save or close the viewer. Headless runs also save `dashboard.png` automatically.

```bash
python main.py --headless --scenario nominal
python main.py --headless --scenario gust --snapshot-at 190
python -m unittest discover -s tests -v
python run_scenarios.py --workers 2
```

`telemetry.csv` holds flight/energy/control data. `events.json` holds transitions and fault reasons. `summary.json` records outcome and unresolved regulatory items. `config_used.json` captures the parameters needed to reproduce the run. The included `example_results/` contains validated example outputs, not flight-test data.

| File | Purpose |
|---|---|
| `main.py` | CLI / launch entry point |
| `physics.py` | Rigid-body motion, motors, drag, battery and ground contact |
| `simulator.py` | Delivery state machine, planning and contingency rules |
| `weather.py` | Climate reference, NWS loader, density, wind and solar-position estimate |
| `mesh.py` | Original STL reader and display-only simplification |
| `viewer.py` | 3D dashboard, buttons, plots and normal-speed landing playback |
| `run_scenarios.py` | Repeatable fault/range/time-step comparisons |
| `MODEL_NOTES.md` | Equations, assumptions, fidelity limits and validation ladder |
| `WEATHER_AND_REGULATIONS.md` | Source provenance and required external operational review |

## 9. Important safety and regulatory boundary

There is no software setting here that authorizes an outdoor flight. Review flags remain **NOT VERIFIED**, and the viewer always labels the application simulation-only. Part 107 numerical screens are illustrative and do not substitute for the applicable operator's certificate, exemptions, airspace approvals or procedures. A 3-mile delivery is not made lawful by keeping altitude below 400 ft.

Read `WEATHER_AND_REGULATIONS.md` before treating this as planning support. This version does not establish safe flight, regulatory compliance, production reliability, aircraft efficiency, or certification.

# SWARM × Flight Ops — full-distance Columbia SITL

The combined dashboard imports the three routes from `flightpath-vscode/js/app.js`
into `simulation/columbia-routes.json`. ArduCopter flies their original geographic
waypoints, lands and disarms at the target, then the owned SITL operator uploads a
second mission, normally arms, climbs, follows the reversed route, lands at pickup,
and disarms. The target landing must be within 2 m before the return leg starts.
The map's aircraft marker follows `GLOBAL_POSITION_INT`; there is no timer-driven
city aircraft animation. The target stop is a simulated delivery visit, **not a
payload drop-off**: pad support and latch/load evidence are unavailable for these
unsurveyed sites, so the package remains retained and no release command is sent.

| Flightpath tracking number | Route | Outbound | Round trip |
|---|---|---:|---:|
| UAV-COLA-2047 | Publix → Finlay Park | 924.169 m | 1,848.339 m |
| UAV-COLA-3186 | Mast General Store → Riverfront Park | 1,448.628 m | 2,897.257 m |
| UAV-COLA-5729 | Publix → Columbia Museum of Art | 1,112.662 m | 2,225.324 m |

Distances are the sum of WGS84 ellipsoidal waypoint legs, calculated with pinned
GeographicLib 2.1, not scaled into the original 80 m test area. Location names and
coordinates are inherited from the presentation project, not surveyed or newly
geocoded. Waypoint cornering means flown GPS track length can differ slightly
from planned polyline length.

## Run

From the Hackathon directory:

```bash
bash flightpath-vscode/start.sh
```

Or from the flight software repository:

```bash
PYTHONPATH=flight-software .venv/bin/python -m simulation.flight_ops_bridge
```

Open `http://127.0.0.1:8765`, enter the bridge access code printed in the terminal,
and click **Connect**. Select a route and click **Start SITL route**. The page
exposes the three named Columbia routes; the bridge still accepts validated custom
GPS destinations through its authenticated API for development tooling. Only one
simulation can run at a time. If the bridge was already running
before this update, stop it in its terminal with Ctrl-C and restart it; the old
process does not reload Python code. Each restart produces a new access code.

The same private dashboard is published at
https://quad-flight-ops.validokapi9.chatgpt.site. Browser restrictions can block
HTTPS-to-loopback requests; use the local dashboard when that happens. No local
simulator executes on the hosting service. VS Code Live Server on localhost:5500
is also allowed by the authenticated bridge. Tokens stay in browser memory.

A command-line example from the repository:

```bash
PYTHONPATH=flight-software .venv/bin/python -m simulation.geographic_flight \
  --request '{"route_id":"UAV-COLA-2047","altitude":43,"speed":5}' \
  --output simulation/runs/columbia-demo
```

## Profile and ownership

The original local demo remains limited to 80 m with its 100 m FC fence. The
separate geographic profile accepts 2–20 coordinates in latitude 33.97–34.04,
longitude −81.08–−81.00, outbound length at most 2 km, altitude 20–80 m above home,
and speed 0.5–5 m/s. A low speed can fail the energy check even inside this area.
These are explicit study limits, not validated aircraft capability.

Each owned SITL process starts at the route's first GPS coordinate. Home elevation
is an **assumed 100 m MSL flat ground**, not real terrain. The generated
`geographic-sitl-only.parm` changes only:

- `FENCE_RADIUS`: maximum home-to-waypoint distance rounded up to 50 m, plus 100 m.
- `FENCE_ALT_MAX`: 100 m.
- `RTL_ALT`: requested relative altitude, converted to centimetres.
- `WPNAV_SPEED`: requested speed, converted to centimetres/second.

Arming checks, fence action, RC/GCS/EKF/battery failsafes remain enabled. Their
critical values are read back before upload. The companion authenticates and
validates the request, matches it to the owned simulator profile, checks health
and loaded round-trip reserve, uploads two missions via MAVLink, monitors
health/return reserve, and verifies each leg's waypoints and both landings.
ArduCopter owns flight control. The simulator operator makes each normal arming
request; the companion API has no arming method.

The 900 W energy proxy and conceptual 222 Wh mapping are unchanged and uncalibrated.
Stock physical battery capacity is unlimited while the FC's configured 10 Ah
monitor tracks consumption; this does not establish real vehicle endurance.
Terrain, buildings, public landing areas, route clearance and physical support
sensors are not modeled. This is full-distance navigation testing, not a physical
city-delivery capability or selected-concept performance prediction.

## Artifacts and sources

Runs include `route.json`, `manifest.json` (home, firmware, exact command and
parameter hashes), `result.json`, MAVLink telemetry/outgoing logs and DataFlash.
The STEP-derived concept GLB is retained from Flight Ops. The authoritative Site
source is `site/flight-ops/dist`; `flightpath-vscode` contains a synchronized static
copy for Live Server, plus the original animation at `presentation.html`.

Implementation references:
[GeographicLib WGS84 API](https://geographiclib.sourceforge.io/html/python/code.html),
[ArduPilot SITL tutorial](https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html),
[ArduPilot cylindrical fence](https://ardupilot.org/copter/docs/common-ac2_simple_geofence.html).

See [TEST_RESULTS.md](TEST_RESULTS.md) for actual executed checks and limitations.

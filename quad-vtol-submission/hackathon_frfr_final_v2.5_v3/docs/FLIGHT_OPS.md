# SWARM flight dashboard

The owner-private Site at https://quad-flight-ops.validokapi9.chatgpt.site uses
the original Flightpath white sidebar, Columbia street map, tracking numbers and
route cards. Select one of the three Columbia routes, connect the local simulator,
then click **Start SITL route**. Each route lands at its target, pauses, climbs
again and returns home. This is a simulated stop with the payload retained;
physical pad support and release are not modeled. The page includes the
STEP-derived concept drone and live graphs for battery percentage, altitude above
home, groundspeed, roll, pitch and heading. Flight mode, battery current and
voltage are updating values. A health panel also shows GPS fix/satellites,
estimator/RC status, armed/landed state, mission item and telemetry age. Its only
flight control starts a selected geographic route. The earlier
local-position and flight-test controls are removed from the page.

The graphs use ArduCopter MAVLink telemetry from the loopback bridge. With no
bridge, the Finlay Park route replays recorded SITL telemetry with a **RECORDED
SITL** label. Selecting another route without a live run shows an empty preview
instead of putting Finlay Park readings on that route. A prior completed bridge
run no longer overwrites replay. An unavailable field stays blank. These are
readings from the stock SITL vehicle, not the selected hardware.

From this repository:

```bash
PYTHONPATH=flight-software .venv/bin/python -m simulation.flight_ops_bridge
```

Or from the parent Hackathon directory:

```bash
bash flightpath-vscode/start.sh
```

Open `http://127.0.0.1:8765`, paste the access code printed in the terminal, and
click **Connect**. Restart any bridge process started before a code change. The
server listens only on `127.0.0.1:8765`; the published HTTPS Site may be blocked
from loopback by browser policy, in which case use the local dashboard. The bridge
starts only an owned SITL process, exposes no serial/hardware or actuator endpoint,
and permits one run at a time. See [GEOGRAPHIC_FLIGHTS.md](GEOGRAPHIC_FLIGHTS.md)
for route limits and simulation assumptions.

The bridge retains its authenticated `/api/position` and `/api/scenario` endpoints
for existing development tooling; this page does not expose those controls. Raw
run evidence is saved under ignored `simulation/runs/flight-ops/<run-id>/`.

Check assets and graph data wiring with:

```bash
node site/flight-ops/check.mjs
node site/flight-ops/check-runtime.mjs
```

See [TEST_RESULTS.md](TEST_RESULTS.md) for the ArduCopter suite and the latest
targeted bridge check. The Site source is `site/flight-ops/dist`; a matching static
copy is in `../flightpath-vscode/`. The original presentation animation remains at
`../flightpath-vscode/presentation.html`.

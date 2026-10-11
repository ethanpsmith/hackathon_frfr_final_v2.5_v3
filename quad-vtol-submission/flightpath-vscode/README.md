# SWARM × Flight Ops

This is the merged Columbia route dashboard, backed by the existing ArduCopter
SITL flight software. The three Flightpath routes now execute at their full WGS84
distances. The map uses actual simulator GPS telemetry; the concept drone uses
the existing STEP-derived GLB.

From the parent Hackathon directory:

```bash
bash flightpath-vscode/start.sh
```

Open http://127.0.0.1:8765, paste the access code printed in the terminal, and
connect. Select one of the three Columbia routes and choose **Start SITL route**.
The page shows live battery percentage, altitude, groundspeed, roll, pitch and
heading graphs. Flight mode, battery current/voltage and health diagnostics
update as values. Every geographic mission lands at the target, pauses, climbs
again, returns to pickup and lands with the payload retained. The target visit
does not actuate a payload release. If an older bridge is already running, stop it with Ctrl-C in its
terminal and restart it. Hardware flight is not implemented.

You can also open this folder in VS Code and use Live Server on port 5500.
The bridge must still run for live simulation. Without a bridge, the dashboard
replays a recorded full-distance Finlay Park SITL flight, explicitly labeled REPLAY.

Published private Site: https://quad-flight-ops.validokapi9.chatgpt.site

The authoritative web source is
`../hackathon_frfr_final_v2.5_v3/site/flight-ops/dist`; this folder is a static copy.
Flight software and simulator setup remain in `../hackathon_frfr_final_v2.5_v3`.
See its `docs/GEOGRAPHIC_FLIGHTS.md` and `docs/TEST_RESULTS.md` for exact commands,
profile limits, test results, and model assumptions.

The original timer-driven presentation is preserved in `presentation.html`
with its original `css/` and `js/` files. Its animation is separate from SITL.

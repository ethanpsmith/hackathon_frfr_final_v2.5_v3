# Quad VTOL submission package

This folder contains the simulation-only Python flight stack, reproducible ArduCopter SITL setup, SWARM dashboard, tests, documentation, a local launcher, and the concept STEP model. The authoritative dashboard is `hackathon_frfr_final_v2.5_v3/site/flight-ops/dist/`; `flightpath-vscode/` is the matching local copy and preserves the original presentation.

ArduCopter C++ source and compiled binaries are third-party dependencies and are **not bundled**. `simulation/setup.sh` downloads the pinned upstream revision and builds the SITL binary in ignored `.tools/`. Setup needs internet access, Git, uv, and a C/C++ compiler. No hardware interface or physical flight procedure is included.

From this folder:

```bash
cd hackathon_frfr_final_v2.5_v3
bash simulation/setup.sh
bash simulation/demo.sh                # disarmed validation
bash simulation/demo.sh --sitl-flight  # explicitly requested simulated flight
bash simulation/test.sh                # SITL and unit tests
```

For the local SWARM dashboard, return to this folder and run `bash flightpath-vscode/start.sh`; open `http://127.0.0.1:8765` and use the bridge code printed in the terminal. The dashboard's static files can also be reviewed without SITL. See the repository's `README.md` and `docs/SETUP.md` for tested versions and limitations. The STEP model in `cad/` is a concept artifact, not the stock SITL dynamics.

`FILE_MANIFEST.sha256` lists every submitted file and its hash. Generated test runs, build outputs, virtual environments, cache files, and credentials are excluded. This package is a folder for submission or a fresh Git repository; it contains no nested Git metadata.

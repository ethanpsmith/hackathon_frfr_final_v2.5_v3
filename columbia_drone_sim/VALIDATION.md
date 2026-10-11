# Verification record — 10 October 2026

This record concerns software behaviour only. No real aircraft was flown or certified.

## Automated checks

21 `unittest` checks passed on Python 3.12.14, NumPy 2.3.5 and Matplotlib 3.10.8. Coverage includes the supplied STL bounds, wind-direction convention, temperature/humidity density effects, battery power equation, initial free fall, hover/attitude normalization, payload release interlocks, delivery/return, dry-weather and altitude gates, energy rejection, blocked-pad handling, fault outcome classification, fixed-step validation, weather units/freshness/missing values, haze-versus-rain parsing, recent-observation sampling, solar position and repeatable seeds.

Command: `python -m unittest discover -s tests -v`

The full scenario sweep covers 12 scenarios plus 2-mile, 5-mile and half-time-step cases (15 runs). It writes raw telemetry and event logs, not just a pass badge.

## Reference outcomes using the included assumptions

| Case | Outcome |
|---|---|
| Nominal, 3 miles each way | Delivered and returned; about 16.94 min; 347.0 Wh modelled chemical energy; 48.1% SOC remaining |
| Crosswind/gust disturbance | Delivered and returned; peak cross-track error about 2.26 m |
| Temporary headwind | Delivered and returned; about 354.0 Wh modelled chemical energy |
| Rain onset | Delivery aborted; package returned to hub |
| 20% motor-capability reduction | Delivery aborted conservatively; returned |
| Complete motor failure | Unsafe impact, reported `CRASHED`; no safe motor-out recovery demonstrated |
| Sudden change to 20% battery charge | Unsafe impact, reported `CRASHED`; this controller/model does not demonstrate recovery from this severe fault |
| Command-link loss | Preprogrammed return with navigation still assumed available |
| Navigation-confidence loss | Virtual local descent; ideal position feedback means this does NOT demonstrate real GNSS-denied recovery |
| Blocked destination pad | Package retained; returned to hub |
| 36°C hot-weather scenario | Delivered and returned under the assumed thermal/aircraft model |
| Fog / 800 m visibility | Dispatch rejected |
| 5-mile one-way request | Dispatch rejected for energy margin |

`COMPLETE` means the vehicle returned to the hub; inspect `delivered` and `aborted` to distinguish a successful delivery from a successful abort. A control system cannot guarantee safe recovery from every fault, especially on a four-motor aircraft.

With a 0.01 s rather than 0.02 s step, the nominal run used about 346.6 Wh versus 347.0 Wh (roughly 0.13% difference). This is a numerical-sensitivity check, not rigorous convergence: the seeded stochastic gust realizations also change with sample interval. It is not a real-world accuracy claim.

## Visual and file checks

- Loaded and rendered the user's STL in the 3D dashboard.
- Original STL preserved byte-for-byte; SHA-256 `8242b03e331f508ca5f28b25d8157265837c2c28847bfa80d163544af7a73ebf`.
- Rendered flight, release and post-release previews with desktop controls constructed.
- Tested headless rendering and GUI widget creation; an actual Windows desktop session was not available for OS-level GUI testing.
- Live NWS retrieval returned HTTP 403 in this environment; fixture-based parser/freshness tests passed. A successful internet retrieval is not claimed.

## Unverified prerequisites

STL units, physical aircraft parameters, motor/propeller selection, battery pack characteristics, true endurance, safe payload release, real-world control performance, terrain/airspace, landing-site risk and legal authority remain unverified. Default weather/flight limits are study assumptions. See `MODEL_NOTES.md` and `WEATHER_AND_REGULATIONS.md` before interpreting any output.

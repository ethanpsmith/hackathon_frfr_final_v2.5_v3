# Model, mathematics and engineering limits

## Scope

This is a reduced-order, six-degree-of-freedom **quadrotor** simulation. There are no fixed wings or tiltrotor transitions. Climb, level cruise and descending cruise use thrust-vector tilt; the aircraft does not switch to fixed-wing lift. The uploaded mesh is visual geometry only. No inferred solid-STL density, mass or structural properties are used.

The controller uses ideal simulated position, velocity and attitude. This deliberately isolates vehicle/weather/mission response; it is not a sensor-fusion implementation. Wind-aware guidance uses the model's mean wind, while random gusts must be rejected by feedback. Real wind estimation is not demonstrated.

## Coordinate system and rigid-body motion

World coordinates are east/north/up (ENU), in metres. Body axes are forward/left/up. Quaternion order is `(w, x, y, z)` and `R(q)` maps body vectors to world vectors.

```
position_dot = velocity
mass * velocity_dot = R * (body_drag + [0,0,total_thrust]) + [0,0,-mass*g]
I * omega_dot = torque - omega × (I * omega)
quaternion_dot = 0.5 * quaternion ⊗ [0, omega]
```

`g = 9.80665 m/s²`. Fixed-step semi-implicit Euler advances velocities before positions; quaternion normalization controls norm drift. This is not a high-order flight-dynamics solver. The included 0.01/0.02 s comparison checks numerical sensitivity, not physical accuracy.

Payload is a point mass at an assumed body offset. The combined centre of gravity and inertia use the parallel-axis theorem. Release occurs only after landing and motor shutdown. Vehicle mass remains constant during battery discharge and changes only at package release.

## Atmosphere and disturbances

```
T(z) = T_surface - 0.0065*z                 [kelvin]
p(z) = p_surface * exp(-g*z / (R_d*T(z)))   [pascal]
rho = (p-e)/(R_d*T) + e/(R_v*T)            [kg/m³]
```

Water-vapour pressure `e` is estimated from relative humidity and a saturation-pressure approximation. The lapse rate and humidity treatment are simplified. Density reduces at higher temperature/elevation and affects both thrust limits and induced-power demand.

```
wind(z) = surface_wind * (max(z,1)/10)^alpha + event + gust
gust_next = exp(-dt/tau)*gust + sigma*sqrt(1-exp(-2*dt/tau))*normal_sample
```

`alpha = 0.16` and `tau = 2.5 s` are assumptions. The process is an Ornstein–Uhlenbeck coloured-noise model, **not validated Dryden/von Kármán turbulence**. Vertical turbulence is one quarter of the horizontal standard deviation. Meteorological wind direction is FROM; 270° wind travels east. The model does not reproduce urban gusts, wake turbulence, terrain-channelled winds, storms or spatially resolved weather. The shear law is especially approximate near the ground.

## Drag, control, and motors

For each body axis:

```
v_air_body = R.T * (velocity - wind)
drag_body_i = -0.5*rho*(Cd*A)_i*v_air_body_i*abs(v_air_body_i)
```

The configuration stores effective `Cd*A`, not a measured coefficient or an STL-computed frontal area. Rotation produces a drag moment when the assumed combined CG differs from the body origin. Linear rotational damping is included.

Guidance slows the ground-velocity reference near each destination, respects assumed climb/descent limits, and caps the commanded horizontal airspeed using wind decomposition. A velocity PI controller creates the required acceleration; the desired thrust direction is tilt-limited. Geometric attitude PD feedback generates torque. An X-quad mixer allocates total thrust and roll/pitch/yaw torques to four motors.

```
T_i,next = T_i + (1-exp(-dt/tau_motor))*(clipped_command_i - T_i)
T_i,max = T_i,reference * (rho/1.225) * (loaded_voltage/full_voltage)^2 * motor_health_i
```

This is a thrust-actuator approximation, not a blade-element propeller model or ESC electrical model. Yaw torque/thrust ratio is assumed. Saturation and motor health are logged. Clipping can reduce control authority; the code does not guarantee tracking, safe descent or motor-out survival.

## Power, battery and reserve

```
P_induced = sum(T_i^(3/2) / sqrt(2*rho*A_rotor))
P_electric = (P_induced + P_drag + max(m*g*v_z,0))/eta + P_profile + P_avionics
P_drag = max(-drag_body dot v_air_body, 0)
V_open = cells*(3.3 + 0.9*SOC)
V_loaded = V_open - I*R_internal
P_electric = V_loaded*I
SOC_dot = -I / (capacity_Ah*3600)
C_thermal*T_battery_dot = I²*R_internal - h*(T_battery-T_ambient)
```

The battery equation is solved using its low-current quadratic root. Negative discriminant is flagged as a power-limit event. Remaining mission energy uses a conservative nominal-voltage equivalent; cumulative chemical energy uses `V_open*I`, so the two values are not identical measures. The fixed initial cold-capacity factor and resistance dependence are illustrative. There is no regeneration during descent.

Hover-induced power is retained during cruise (no translational-lift benefit). Drag and climb work are added. This is intentionally a low-fidelity sizing approximation, **not a guaranteed conservative bound**: real vortex-ring behaviour, interference, thermal/current limits, descent aerodynamics and propeller efficiencies can make performance worse. The linear battery curve is also not a chemistry-specific validation.

The planning estimator checks loaded outbound, empty return, and the loaded-abort return case, adds a 10% planning loss allowance plus phase-time allowances, then requires the configured reserve. During flight it recomputes return energy every 0.5 s. It is a screening heuristic, not an optimal-control or guaranteed reachable-set solution. Emergency landing safety is not established by an energy calculation.

## Delivery/contact model

Terrain is a perfectly flat, level rigid surface. A contact plane at the assumed landing-gear height stops a successful landing. A hard vertical impact (>0.85 m/s), excessive lateral speed (>1.0 m/s), or tilt (>15°) flags a crash immediately. These are illustrative thresholds, not validated landing-gear strength limits. There is no landing-gear compliance, bounce, skid friction, ground effect or finite-element impact model.

The loaded aircraft lands, stops its motors, waits 10 seconds, spends 3 seconds releasing cargo, and confirms separation for 6 seconds. The package mass and inertia contribution then disappear from the aircraft. If an occupied pad is reported while grounded, it stays stopped for recovery. No cable dynamics are present.

## Deliberate limitations

- No CFD, measured aerodynamic derivatives, rotor RPM/current maps, propwash or structural analysis.
- No real terrain/buildings, obstacles, people, vehicles, aircraft traffic, DAA, ADS-B or Remote ID broadcast.
- No GPS/IMU noise, EKF, magnetic interference, vibration or physical GNSS-denied estimator; navigation/link failures are mission-level injections.
- No actual communications latency, encryption, software assurance or autopilot hardware interface.
- No automatic airspace/TFR/NOTAM/LAANC lookup or legal approval.
- No rainfall damage, icing accretion or lightning strike physics: precipitation/weather hazards deny or abort the mission.
- No real warehouse loading hardware, latch, vision-based pad detection or package-separation sensor.
- No economic claim or validated manufacturer range. A successful simulation is not evidence the uploaded design can fly.

## How to mature this into a trustworthy engineering tool

First measure component masses, thrust/power curves and pack discharge behaviour. Fit the parameters against bench data. Check hover and restrained tests through an authorized engineering test programme, then compare logged flight tracks/energy/temperatures with simulated ones. Add sensor and communications models and uncertainty ranges. Validate the controller using the selected autopilot's software-/hardware-in-the-loop environment before any independent flight tests. Have the qualified operator define routes, contingencies, staffing and approvals.

Treat each added capability as a separate validation step. The educational controller in this package must not be copied onto a real drone as production flight software.

#!/usr/bin/env python3


import argparse
import csv
import math
import time

import numpy as np

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    p = None


# ======================== SETTINGS YOU CAN CHANGE ========================
DT = 0.1 / 240                 # Your requested PHYSICS step, unchanged.
CONTROL_EVERY = 10             # Controller runs every 10 physics steps: 240 Hz.
MASS = 1.0                    # Total kg; visual links have no extra mass.
SPAN = 0.886                  # Total X/Y footprint, including rotor discs.
ROTOR_RADIUS = 0.095          # Assumed propeller radius, m.
ROTOR_OFFSET = SPAN / 2 - ROTOR_RADIUS
ROTOR_Z = 0.055
INERTIA = np.array([0.045, 0.045, 0.080])  # Assumed kg m^2 about body axes.
MAX_ROTOR_THRUST = 8.0         # N per rotor; total maximum 32 N.
MOTOR_TIME_CONSTANT = 0.035    # Seconds, first-order thrust response.
YAW_TORQUE_PER_THRUST = 0.02   # N m / N (assumed reaction torque coefficient).
MAX_TILT = math.radians(30)
MAX_XY_SPEED = 1.2            # m/s
MAX_Z_SPEED = 0.8             # m/s
ARRIVAL_RADIUS = 0.15         # m
ARRIVAL_SPEED = 0.20          # m/s; passing through fast does not count.
ARRIVAL_DWELL = 0.4           # Seconds settled before advancing.
GRAVITY = np.array([0.0, 0.0, -9.81])
START_POSITION = [0.0, 0.0, 0.25]

WAYPOINTS = np.array([
    [0.0, 0.0, 1.5],
    [1.0, 1.0, 1.5],
    [1.0, 1.0, 0.2],
    [1.0, 1.0, 1.5],
    [0.0, 0.0, 1.5],
    [0.0, 0.0, 0.2],
])

MEAN_WIND = np.array([2.0, -0.8, 0.0])  # Wind velocity, m/s.
GUST_RANGE = np.array([1.3, 1.3, 0.3])  # Random velocity changes, m/s.
RAIN_LOAD = 0.20                         # N downward; 0 disables rain proxy.
WEATHER_POINT = np.array([0.05, -0.04, 0.02])  # Body point; causes some torque.

# All four positions are relative to the body's centre of mass.
ROTOR_POSITIONS = np.array([
    [ ROTOR_OFFSET,  ROTOR_OFFSET, ROTOR_Z],  # 0 front-left
    [ ROTOR_OFFSET, -ROTOR_OFFSET, ROTOR_Z],  # 1 front-right
    [-ROTOR_OFFSET,  ROTOR_OFFSET, ROTOR_Z],  # 2 rear-left
    [-ROTOR_OFFSET, -ROTOR_OFFSET, ROTOR_Z],  # 3 rear-right
])
# Signs of reaction torque ON THE BODY; opposite corners share a sign.
REACTION_SIGNS = np.array([1.0, -1.0, -1.0, 1.0])


def custom_external_force(sim_time, position, velocity):
    """EDIT HERE: return any extra WORLD force [Fx, Fy, Fz] in newtons.

    This runs every physics step. The controller is not told this force.
    Example: a 3 N sideways push from simulation time 8 to 9 seconds:
        if 8.0 <= sim_time < 9.0:
            return np.array([3.0, 0.0, 0.0])
    For random gusts, keep a seeded generator/state across calls (Weather below).
    Holding a random value for a fraction of a second is easier to study than
    drawing a different value at all 2400 physics steps per second.
    """
    return np.zeros(3)


def limit_length(vector, maximum):
    length = np.linalg.norm(vector)
    return vector * min(1.0, maximum / max(length, 1e-12))


# ================================ CONTROL ================================
class Controller:
    def __init__(self):
        self.previous_velocity = None
        self.acceleration = np.zeros(3)
        self.disturbance_acceleration = np.zeros(3)
        # [total thrust, roll torque, pitch torque, yaw torque] = mixer @ thrusts.
        # Torque from a vertical force: r x F = [y*T, -x*T, 0].
        self.mixer = np.vstack([
            np.ones(4), ROTOR_POSITIONS[:, 1], -ROTOR_POSITIONS[:, 0],
            REACTION_SIGNS * YAW_TORQUE_PER_THRUST,
        ])
        self.inverse_mixer = np.linalg.inv(self.mixer)
        self.allocation_scale = 1.0

    def update(self, target, position, velocity, rotation, angular_velocity,
               expected_acceleration, sample_dt):
        # 1. Infer acceleration from velocity feedback. Compare with the motion
        # predicted from LAST interval's applied motor thrust and gravity.
        # Low-pass the difference to estimate unmodelled forces (wind, pushes).
        if self.previous_velocity is not None:
            raw_acceleration = (velocity - self.previous_velocity) / sample_dt
            alpha_a = 1.0 - math.exp(-sample_dt / 0.05)
            self.acceleration += alpha_a * (raw_acceleration - self.acceleration)
            if position[2] > 0.12 and np.linalg.norm(raw_acceleration) < 50:
                residual = limit_length(raw_acceleration - expected_acceleration, 5.0)
                alpha_d = 1.0 - math.exp(-sample_dt / 0.30)
                self.disturbance_acceleration += alpha_d * (
                    residual - self.disturbance_acceleration)
        self.previous_velocity = velocity.copy()

        # 2. Position error -> desired velocity, with speed limits.
        desired_velocity = 1.3 * (target - position)
        desired_velocity[:2] = limit_length(desired_velocity[:2], MAX_XY_SPEED)
        desired_velocity[2] = np.clip(desired_velocity[2], -MAX_Z_SPEED, MAX_Z_SPEED)

        # 3. Velocity error -> desired acceleration; oppose estimated disturbance.
        acceleration_command = 2.8 * (desired_velocity - velocity)
        acceleration_command -= self.disturbance_acceleration
        acceleration_command[2] = np.clip(acceleration_command[2], -4.0, 5.0)
        desired_force = MASS * (acceleration_command - GRAVITY)
        desired_force[:2] = limit_length(
            desired_force[:2], desired_force[2] * math.tan(MAX_TILT))

        # 4. Desired body +Z points along the requested force. Keep yaw near zero.
        # These are DESIRED axes; we never directly set the body's orientation.
        desired_z = desired_force / np.linalg.norm(desired_force)
        desired_y = np.cross(desired_z, [1.0, 0.0, 0.0])
        desired_y /= np.linalg.norm(desired_y)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))

        # 5. Orientation error -> BODY torque. Angular rate damps rotation.
        skew_error = 0.5 * (desired_rotation.T @ rotation
                            - rotation.T @ desired_rotation)
        angle_error = np.array([skew_error[2, 1], skew_error[0, 2], skew_error[1, 0]])
        body_rates = rotation.T @ angular_velocity
        angular_acceleration = -np.array([90., 90., 50.]) * angle_error
        angular_acceleration -= np.array([16., 16., 12.]) * body_rates
        torque = INERTIA * angular_acceleration + np.cross(body_rates, INERTIA * body_rates)
        total_thrust = max(0.0, float(desired_force @ rotation[:, 2]))

        # 6. Motor mixing: four individual upward thrust commands.
        # If necessary, reduce torque demands together to respect motor limits
        # while preserving the feasible requested collective thrust.
        collective = np.clip(total_thrust / 4.0, 0.0, MAX_ROTOR_THRUST)
        differential = self.inverse_mixer @ np.r_[0.0, torque]
        scale = 1.0
        for delta in differential:
            if delta > 1e-12:
                scale = min(scale, (MAX_ROTOR_THRUST - collective) / delta)
            elif delta < -1e-12:
                scale = min(scale, -collective / delta)
        self.allocation_scale = max(0.0, scale)
        return np.clip(collective + self.allocation_scale * differential,
                       0.0, MAX_ROTOR_THRUST)


class Weather:
    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)
        self.gust = np.zeros(3)
        self.target_gust = np.zeros(3)
        self.next_gust = 0.0

    def force(self, sim_time, velocity):
        if sim_time >= self.next_gust:
            self.target_gust = self.rng.uniform(-GUST_RANGE, GUST_RANGE)
            self.next_gust = sim_time + 2.0
        self.gust += (1.0 - math.exp(-DT / 0.4)) * (self.target_gust - self.gust)
        air_relative_velocity = MEAN_WIND + self.gust - velocity
        # F = 1/2 rho CdA |v_rel| v_rel, with assumed rho=1.225 and CdA=0.08.
        wind_force = 0.5 * 1.225 * 0.08 * np.linalg.norm(air_relative_velocity) * air_relative_velocity
        rain_force = np.array([0., 0., -RAIN_LOAD])
        return wind_force + rain_force


# ============================= PYBULLET MODEL =============================
def create_drone():
    body_half = [0.09, 0.09, 0.04]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=body_half)
    body = p.createVisualShape(p.GEOM_BOX, halfExtents=body_half,
                               rgbaColor=[0.20, 0.23, 0.29, 1])
    arm = p.createVisualShape(p.GEOM_BOX,
        halfExtents=[math.sqrt(2) * ROTOR_OFFSET, 0.018, 0.012],
        rgbaColor=[0.15, 0.17, 0.20, 1])
    # Front rotors blue, rear rotors orange to make attitude easy to see.
    front = p.createVisualShape(p.GEOM_CYLINDER, radius=ROTOR_RADIUS,
                                length=0.018, rgbaColor=[0.10, 0.60, 0.95, 1])
    rear = p.createVisualShape(p.GEOM_CYLINDER, radius=ROTOR_RADIUS,
                               length=0.018, rgbaColor=[1.0, 0.48, 0.08, 1])
    identity = [0., 0., 0., 1.]
    drone = p.createMultiBody(
        baseMass=MASS, baseCollisionShapeIndex=collision, baseVisualShapeIndex=body,
        basePosition=START_POSITION,
        linkMasses=[0.] * 6, linkCollisionShapeIndices=[-1] * 6,
        linkVisualShapeIndices=[arm, arm, front, front, rear, rear],
        linkPositions=[[0., 0., 0.], [0., 0., 0.]] + ROTOR_POSITIONS.tolist(),
        linkOrientations=[p.getQuaternionFromEuler([0, 0, math.pi / 4]),
                          p.getQuaternionFromEuler([0, 0, -math.pi / 4])] + [identity] * 4,
        linkInertialFramePositions=[[0., 0., 0.]] * 6,
        linkInertialFrameOrientations=[identity] * 6,
        linkParentIndices=[0] * 6, linkJointTypes=[p.JOINT_FIXED] * 6,
        linkJointAxis=[[0., 0., 1.]] * 6)
    # Model the entire mass distribution in the base inertia, rather than using
    # just the small centre box's inertia. Attached links are decorative.
    p.changeDynamics(drone, -1, localInertiaDiagonal=INERTIA.tolist(),
                     linearDamping=0., angularDamping=0.)
    return drone


def apply_rotor_thrusts(drone, position, rotation, thrusts):
    up = rotation[:, 2]  # THIRD COLUMN = body +Z expressed in world coordinates.
    for i, thrust in enumerate(thrusts):
        rotor_world_position = position + rotation @ ROTOR_POSITIONS[i]
        # FOUR SEPARATE FORCE CALLS, at the four distinct rotor locations.
        p.applyExternalForce(drone, -1, (up * thrust).tolist(),
                             rotor_world_position.tolist(), p.WORLD_FRAME)
        # Rotor spin reaction supplies yaw torque; roll/pitch arise from r x F.
        reaction = up * (REACTION_SIGNS[i] * YAW_TORQUE_PER_THRUST * thrust)
        p.applyExternalTorque(drone, -1, reaction.tolist(), p.WORLD_FRAME)


# ================================== RUN ==================================
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--headless', action='store_true', help='Run without a window or pacing.')
    parser.add_argument('--seconds', type=float, default=0, help='Simulated duration; 0 runs until closed.')
    parser.add_argument('--no-weather', action='store_true', help='Disable built-in wind and rain load.')
    parser.add_argument('--seed', type=int, default=7, help='Seed for reproducible random gusts.')
    parser.add_argument('--csv', help='Optional telemetry CSV path.')
    args = parser.parse_args()
    if p is None:
        raise SystemExit('Install dependencies: python3 -m pip install pybullet numpy')
    if args.seconds < 0 or not np.isfinite(args.seconds):
        parser.error('--seconds must be a finite, nonnegative number')
    if args.headless and args.seconds == 0:
        args.seconds = 45.0
    if len(WAYPOINTS) == 0:
        raise SystemExit('Add at least one waypoint to WAYPOINTS.')

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*GRAVITY)
    p.setTimeStep(DT)
    p.setRealTimeSimulation(0)
    p.loadURDF('plane.urdf')
    drone = create_drone()
    if not args.headless:
        if hasattr(p, 'COV_ENABLE_MOUSE_PICKING'):
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
        p.resetDebugVisualizerCamera(4.0, 45, -28, [0.5, 0.5, 0.8])
        for i, waypoint in enumerate(WAYPOINTS):
            p.addUserDebugText(str(i + 1), waypoint.tolist(), [0.1, 0.5, 0.1], 1.2)

    controller = Controller()
    weather = Weather(args.seed)
    weather_enabled = not args.no_weather
    commands = np.full(4, MASS * 9.81 / 4)
    actual_thrusts = commands.copy()
    expected_delta_velocity = np.zeros(3)
    waypoint_index, dwell, route_complete = 0, 0.0, False
    push_until = -1.0
    control_dt = DT * CONTROL_EVERY
    motor_alpha = 1.0 - math.exp(-DT / MOTOR_TIME_CONSTANT)
    start_wall_time = time.perf_counter()
    status_id = -1
    step = 0
    peak_tilt, minimum_height = 0.0, float('inf')
    stream = open(args.csv, 'w', newline='') if args.csv else None
    writer = csv.writer(stream) if stream else None
    if writer:
        writer.writerow(['time_s', 'waypoint', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                         'ax', 'ay', 'az', 'roll_rad', 'pitch_rad', 'yaw_rad',
                         'T0_N', 'T1_N', 'T2_N', 'T3_N', 'weather_Fx', 'weather_Fy',
                         'weather_Fz', 'estimated_Fx', 'estimated_Fy', 'estimated_Fz'])
    print('Four-rotor simulation. P: sideways push. W: toggle weather. Ctrl+C: exit.')

    try:
        while p.isConnected(client):
            sim_time = step * DT
            if args.seconds and sim_time >= args.seconds:
                break
            pos_tuple, quaternion = p.getBasePositionAndOrientation(drone)
            vel_tuple, omega_tuple = p.getBaseVelocity(drone)
            position, velocity = np.array(pos_tuple), np.array(vel_tuple)
            omega = np.array(omega_tuple)
            rotation = np.array(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)

            if step % CONTROL_EVERY == 0:
                target = WAYPOINTS[waypoint_index]
                if not route_complete:
                    arrived = (np.linalg.norm(target - position) < ARRIVAL_RADIUS
                               and np.linalg.norm(velocity) < ARRIVAL_SPEED)
                    dwell = dwell + control_dt if arrived else 0.0
                    if dwell >= ARRIVAL_DWELL:
                        print(f'{sim_time:6.2f}s: reached waypoint {waypoint_index + 1}/{len(WAYPOINTS)}')
                        dwell = 0.0
                        if waypoint_index + 1 < len(WAYPOINTS):
                            waypoint_index += 1
                        else:
                            route_complete = True
                            print('Route complete; continuing to hold the final position.')
                commands = controller.update(WAYPOINTS[waypoint_index], position, velocity,
                    rotation, omega, expected_delta_velocity / control_dt, control_dt)
                expected_delta_velocity[:] = 0.0

            # Actual motor response takes time; it does not jump to the command.
            actual_thrusts += motor_alpha * (commands - actual_thrusts)
            apply_rotor_thrusts(drone, position, rotation, actual_thrusts)
            expected_delta_velocity += DT * (GRAVITY + rotation[:, 2] * actual_thrusts.sum() / MASS)

            # External forces are injected separately. They are NOT supplied to
            # the controller, which learns their effect from measured motion.
            weather_force = weather.force(sim_time, velocity) if weather_enabled else np.zeros(3)
            weather_world_point = position + rotation @ WEATHER_POINT
            p.applyExternalForce(drone, -1, weather_force.tolist(),
                                 weather_world_point.tolist(), p.WORLD_FRAME)
            extra_force = np.asarray(custom_external_force(sim_time, position, velocity), dtype=float)
            if sim_time < push_until:
                extra_force = extra_force + np.array([3., 0., 0.])
            p.applyExternalForce(drone, -1, extra_force.tolist(), position.tolist(), p.WORLD_FRAME)

            tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1, 1))))
            peak_tilt = max(peak_tilt, tilt)
            minimum_height = min(minimum_height, position[2])
            if not np.all(np.isfinite(position)) or position[2] < 0.05 or tilt > 75:
                print('Flight left the controller operating range. Reduce disturbances or retune.')
                break

            # 10 Hz telemetry, instead of flooding the terminal every physics step.
            if step % 240 == 0:
                euler = p.getEulerFromQuaternion(quaternion)
                if writer:
                    writer.writerow([sim_time, waypoint_index + 1, *position, *velocity,
                        *controller.acceleration, *euler, *actual_thrusts, *weather_force,
                        *(MASS * controller.disturbance_acceleration)])
                if not args.headless:
                    state = 'HOLD' if route_complete else f'Waypoint {waypoint_index + 1}/{len(WAYPOINTS)}'
                    label = (f'{state} | weather {"ON" if weather_enabled else "OFF"} | tilt {tilt:.1f} deg\n'
                             f'Thrust N: {actual_thrusts.round(2)} | P: push | W: weather')
                    status_id = p.addUserDebugText(label, [0, 0, 2.8], [0.1, 0.1, 0.1],
                                                   textSize=1.0, replaceItemUniqueId=status_id)

            p.stepSimulation()
            step += 1
            # Pace in groups of physics steps (~60 Hz) rather than tiny OS sleeps.
            if not args.headless and step % 40 == 0:
                keys = p.getKeyboardEvents()
                if keys.get(ord('p'), 0) & p.KEY_WAS_TRIGGERED:
                    push_until = step * DT + 0.5
                if keys.get(ord('w'), 0) & p.KEY_WAS_TRIGGERED:
                    weather_enabled = not weather_enabled
                remaining = start_wall_time + step * DT - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if stream:
            stream.close()
        if p.isConnected(client):
            final_position, _ = p.getBasePositionAndOrientation(drone)
            error = np.linalg.norm(np.array(final_position) - WAYPOINTS[waypoint_index])
            print(f'Final error to active target: {error:.3f} m | peak tilt: {peak_tilt:.1f} deg | '
                  f'min height: {minimum_height:.3f} m | route complete: {route_complete}')
            p.disconnect(client)


if __name__ == '__main__':
    main()

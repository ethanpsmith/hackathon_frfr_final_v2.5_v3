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



DT = 1 / 240                  # Physics step, seconds.
CONTROL_EVERY = 10            # Run the flight controller every 10 physics steps.
TELEMETRY_HZ = 10             # CSV samples per simulated second.
TELEMETRY_EVERY = max(1, round(1.0 / (TELEMETRY_HZ * DT)))
RECORD_FPS = 30
RECORD_WIDTH = 960
RECORD_HEIGHT = 540
RECORD_EVERY = max(1, round(1.0 / (RECORD_FPS * DT)))
MASS = 5.4                    # kg
SPAN = 0.886
ROTOR_RADIUS = 0.095
ROTOR_OFFSET = SPAN / 2 - ROTOR_RADIUS
ROTOR_Z = 0.055
INERTIA = np.array([0.36, 0.36, 0.71])  # Rough 5.4 kg, 0.886 m-square estimate.
MAX_ROTOR_THRUST = 100.0       # N per rotor; 400 N total simulator limit.
MOTOR_TIME_CONSTANT = 0.035    # Seconds, first-order thrust response.
YAW_TORQUE_PER_THRUST = 0.02   # N m / N (assumed reaction torque coefficient).
MAX_TILT = math.radians(30)
MAX_XY_SPEED = 4.0            # m/s; gives the drone room to turn around obstacles.
MAX_Z_SPEED = 1.5             # m/s
ARRIVAL_RADIUS = 0.15         # m
ARRIVAL_SPEED = 0.20          # m/s; passing through fast does not count.
ARRIVAL_DWELL = 0.4           # Seconds settled before advancing.
GRAVITY = np.array([0.0, 0.0, -9.81])
START_POSITION = [0.0, 0.0, 0.25]

# Nose-mounted, gimballed depth-camera and avoidance settings.  The camera is
# physically on the drone but levels itself and points toward the active route.
CAMERA_WIDTH = 160
CAMERA_HEIGHT = 120
CAMERA_FOV = 120.0
CAMERA_NEAR = 0.05
CAMERA_FAR = 12.0
CAMERA_EVERY = 20             # 12 camera frames/s at 240 Hz physics.
AVOID_START_DISTANCE = 7.0    # Begin steering away early enough to turn safely.
AVOID_STOP_DISTANCE = 3.0     # Strongest braking/side-stepping distance.
AVOID_MAX_SPEED = 5.0         # m/s velocity bias supplied by the camera.

WAYPOINTS = np.array([
    [0.0, 0.0, 1.5],
    [100.0, 100.0, 1.5],
    [100.0, 100.0, 0.25],
    [100.0, 100.0, 1.5],
    [0.0, 0.0, 1.5],
    [0.0, 0.0, 0.25],
])

# [centre position, half-size] for static, collidable boxes.  They sit on the
# diagonal flight leg so the depth camera has to guide the drone around them.
OBSTACLE_LAYOUT = [
    ([8.0, 8.0, 1.10], [0.45, 0.45, 1.10]),
    ([28.0, 28.0, 1.25], [0.55, 0.55, 1.25]),
    ([55.0, 55.0, 1.00], [0.50, 0.50, 1.00]),
]

MEAN_WIND = np.array([2.0, -0.8, 0.0])  # Average wind velocity, m/s.
GUST_RANGE = np.array([1.3, 1.3, 0.3])  # Slow gust velocity range, m/s.
TURBULENCE_RANGE = np.array([0.35, 0.35, 0.18])  # Faster turbulence velocity range, m/s.
TURBULENCE_INTERVAL = 0.18              # Seconds between turbulence updates.
RAIN_LOAD = 0.20                        # Mean downward rain load, N.
RAIN_VARIATION = 0.10                   # Random rain-load variation, N.
RAIN_LATERAL_FORCE = 0.08               # Small lateral rain disturbance, N.
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
    # Leave this at zero during normal obstacle-avoidance runs.  Press P in
    # the GUI to inject a short 3 N sideways push instead.
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
               expected_acceleration, sample_dt, avoidance_velocity):
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
        desired_velocity = 1.3 * (target - position) + avoidance_velocity
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


def follow_drone_camera(position, quaternion):
    # Camera position relative to the drone
    camera_distance = 3.0
    camera_yaw = 45
    camera_pitch = -25

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=position.tolist(),
    )


def render_chase_frame(position, rotation):
    """Render an RGB chase-camera frame for MP4 recording."""
    forward = rotation[:, 0]
    forward = forward / max(np.linalg.norm(forward), 1e-12)
    eye = position - 5.5 * forward + np.array([0.0, 0.0, 2.4])
    target = position + np.array([0.0, 0.0, 0.35])
    view = p.computeViewMatrix(eye.tolist(), target.tolist(), [0.0, 0.0, 1.0])
    projection = p.computeProjectionMatrixFOV(
        fov=60.0,
        aspect=RECORD_WIDTH / RECORD_HEIGHT,
        nearVal=0.1,
        farVal=250.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        RECORD_WIDTH,
        RECORD_HEIGHT,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
    )
    return np.asarray(rgba, dtype=np.uint8).reshape(
        RECORD_HEIGHT, RECORD_WIDTH, 4)[:, :, :3]


def drone_camera_depth(drone, ground, position, rotation, route_direction):
    """Render the nose camera pointed along the current route direction."""
    # The camera stays mounted ahead of the propellers, but its gimbal turns
    # horizontally toward the waypoint.  Without this, the drone slides on a
    # diagonal route while the fixed camera stares along world +X.
    eye = position + rotation @ np.array([ROTOR_OFFSET + 0.12, 0.0, 0.10])
    forward = np.asarray(route_direction, dtype=float).copy()
    forward[2] = 0.0
    if np.linalg.norm(forward) < 1e-6:
        forward = rotation[:, 0].copy()
        forward[2] = 0.0
    forward /= max(np.linalg.norm(forward), 1e-12)
    up = np.array([0.0, 0.0, 1.0])  # Level stabilized camera.
    target = eye + CAMERA_FAR * forward
    view = p.computeViewMatrix(eye.tolist(), target.tolist(), up.tolist())
    projection = p.computeProjectionMatrixFOV(
        fov=CAMERA_FOV,
        aspect=CAMERA_WIDTH / CAMERA_HEIGHT,
        nearVal=CAMERA_NEAR,
        farVal=CAMERA_FAR,
    )
    _, _, _, depth_buffer, segmentation = p.getCameraImage(
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
        flags=getattr(p, 'ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX', 0),
    )
    depth_buffer = np.asarray(depth_buffer, dtype=float).reshape(
        CAMERA_HEIGHT, CAMERA_WIDTH)
    segmentation = np.asarray(segmentation, dtype=np.int64).reshape(
        CAMERA_HEIGHT, CAMERA_WIDTH)
    # Segmentation stores the object unique ID in the low 24 bits.  Ignore
    # the drone itself and the plane so neither the rotors nor the ground can
    # be classified as an obstacle while the vehicle pitches or rolls.
    object_ids = segmentation & ((1 << 24) - 1)
    depth_buffer[(object_ids == drone) | (object_ids == ground)] = 1.0
    # PyBullet returns a normalized OpenGL depth buffer.  Convert it to the
    # real distance along the camera viewing ray.
    depth_meters = CAMERA_FAR * CAMERA_NEAR / (
        CAMERA_FAR - (CAMERA_FAR - CAMERA_NEAR) * depth_buffer
    )
    return depth_meters, forward


def camera_avoidance_velocity(depth_meters, position, camera_forward, locked_side):
    """Turn the forward depth image into a world-frame velocity bias.

    The controller still owns the flight path.  This returns zero until an
    obstacle enters the central part of the camera image, then brakes and
    sidesteps toward the visually clearer side.
    """
    # Close to the floor the depth image is mostly the ground plane, not an
    # obstacle.  Let the normal take-off/landing controller operate there.
    if position[2] < 0.70:
        return np.zeros(3), float('inf'), 0.0

    h, w = depth_meters.shape
    # Ignore the very top/bottom of the image.  The lower edge can see ground
    # during a modest pitch angle; the upper edge is normally open sky.
    view = depth_meters[int(0.25 * h):int(0.65 * h), int(0.05 * w):int(0.95 * w)]
    left = view[:, :view.shape[1] // 2]
    right = view[:, view.shape[1] // 2:]

    # A low percentile notices a real obstacle without overreacting to one
    # bad depth pixel.  Smaller values mean an object is closer.
    front_distance = float(np.percentile(view, 5))
    left_distance = float(np.percentile(left, 5))
    right_distance = float(np.percentile(right, 5))
    if front_distance >= AVOID_START_DISTANCE:
        return np.zeros(3), front_distance, 0.0

    strength = np.clip(
        (AVOID_START_DISTANCE - front_distance)
        / (AVOID_START_DISTANCE - AVOID_STOP_DISTANCE),
        0.0,
        1.0,
    )

    # Image-left is perpendicular to the camera's route-facing direction.
    # Lock that choice until the box is clear so small depth changes cannot
    # make the drone repeatedly swap left/right escape paths.
    side = locked_side or (1.0 if left_distance > right_distance else -1.0)
    camera_left = np.cross([0.0, 0.0, 1.0], camera_forward)
    camera_left /= max(np.linalg.norm(camera_left), 1e-12)
    avoidance_velocity = (
        -AVOID_MAX_SPEED * strength * camera_forward
        + side * AVOID_MAX_SPEED * strength * camera_left
    )
    return avoidance_velocity, front_distance, side

class Weather:
    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)
        self.gust = np.zeros(3)
        self.target_gust = np.zeros(3)
        self.next_gust = 0.0
        self.turbulence = np.zeros(3)
        self.target_turbulence = np.zeros(3)
        self.next_turbulence = 0.0
        self.rain_load = RAIN_LOAD
        self.next_rain_update = 0.0

    def force(self, sim_time, velocity):
        if sim_time >= self.next_gust:
            self.target_gust = self.rng.uniform(-GUST_RANGE, GUST_RANGE)
            self.next_gust = sim_time + 2.0
        self.gust += (1.0 - math.exp(-DT / 0.4)) * (self.target_gust - self.gust)

        # Faster, smaller turbulence changes the local air velocity.
        if sim_time >= self.next_turbulence:
            self.target_turbulence = self.rng.uniform(
                -TURBULENCE_RANGE, TURBULENCE_RANGE
            )
            self.next_turbulence = sim_time + TURBULENCE_INTERVAL
        turbulence_alpha = 1.0 - math.exp(-DT / 0.07)
        self.turbulence += turbulence_alpha * (
            self.target_turbulence - self.turbulence
        )

        air_relative_velocity = MEAN_WIND + self.gust + self.turbulence - velocity
        # F = 1/2 rho CdA |v_rel| v_rel, with assumed rho=1.225 and CdA=0.08.
        wind_force = 0.5 * 1.225 * 0.08 * np.linalg.norm(air_relative_velocity) * air_relative_velocity

        # Rain is represented by a slowly varying downward load plus a tiny
        # lateral component from drops carried by the wind.
        if sim_time >= self.next_rain_update:
            self.rain_load = max(
                0.0,
                RAIN_LOAD + self.rng.uniform(-RAIN_VARIATION, RAIN_VARIATION),
            )
            self.next_rain_update = sim_time + 0.25
        rain_force = np.array([
            RAIN_LATERAL_FORCE * math.tanh(air_relative_velocity[0]),
            RAIN_LATERAL_FORCE * math.tanh(air_relative_velocity[1]),
            -self.rain_load,
        ])
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


def add_obstacle(position, half_extents):
    """Create one fixed box that the drone can see and collide with."""
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[0.88, 0.16, 0.10, 1.0],
    )
    return p.createMultiBody(
        baseMass=0.0,  # Static world object.
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position,
    )


def create_obstacle_course():
    return [add_obstacle(position, half_extents)
            for position, half_extents in OBSTACLE_LAYOUT]


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
    parser.add_argument('--no-obstacles', action='store_true',
                        help='Disable the obstacle course and camera avoidance.')
    parser.add_argument('--seed', type=int, default=7, help='Seed for reproducible random gusts.')
    parser.add_argument('--csv', help='Optional telemetry CSV path.')
    parser.add_argument('--record', metavar='FILE.mp4',
                        help='Record a 30 FPS chase-camera MP4.')
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
    ground = p.loadURDF('plane.urdf')
    drone = create_drone()
    obstacles_enabled = not args.no_obstacles
    obstacle_ids = create_obstacle_course() if obstacles_enabled else []
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
    avoidance_velocity = np.zeros(3)
    camera_obstacle_distance = float('inf')
    avoidance_side = 0.0
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
    video_writer = None
    cv2 = None
    if args.record:
        try:
            import cv2 as cv2_module
            cv2 = cv2_module
        except ImportError:
            if stream:
                stream.close()
            p.disconnect(client)
            raise SystemExit('MP4 recording needs OpenCV. Install it with: python3 -m pip install opencv-python')
        video_writer = cv2.VideoWriter(
            args.record,
            cv2.VideoWriter_fourcc(*'mp4v'),
            RECORD_FPS,
            (RECORD_WIDTH, RECORD_HEIGHT),
        )
        if not video_writer.isOpened():
            if stream:
                stream.close()
            p.disconnect(client)
            raise SystemExit(f'Could not open MP4 output: {args.record}')
        print(f'Recording chase view to {args.record} at {RECORD_FPS} FPS.')
    if writer:
        writer.writerow(['time_s', 'waypoint', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                         'ax', 'ay', 'az', 'roll_rad', 'pitch_rad', 'yaw_rad',
                         'T0_N', 'T1_N', 'T2_N', 'T3_N', 'weather_Fx', 'weather_Fy',
                         'weather_Fz', 'total_thrust_N', 'weather_force_N',
                         'estimated_Fx', 'estimated_Fy', 'estimated_Fz',
                         'camera_obstacle_m', 'avoid_vx', 'avoid_vy', 'avoid_vz'])
    print('Four-rotor simulation. P: sideways push. W: toggle weather. Ctrl+C: exit.')
    if obstacles_enabled:
        print(f'Camera avoidance enabled for {len(obstacle_ids)} obstacles. '
              'Use --no-obstacles for the baseline route.')

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

            # Capture at a fixed simulated frame rate in both GUI and headless
            # mode.  Convert PyBullet's RGB image to OpenCV's BGR ordering.
            if video_writer is not None and step % RECORD_EVERY == 0:
                rgb = render_chase_frame(position, rotation)
                video_writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

            # Render the actual body-mounted camera at 12 Hz.  The stabilized
            # gimbal tracks the active waypoint, so it sees obstacles ahead of
            # the route even when the drone translates sideways.
            if obstacles_enabled and step % CAMERA_EVERY == 0:
                route_direction = WAYPOINTS[waypoint_index] - position
                depth_meters, camera_forward = drone_camera_depth(
                    drone, ground, position, rotation, route_direction)
                (avoidance_velocity,
                 camera_obstacle_distance,
                 avoidance_side) = camera_avoidance_velocity(
                    depth_meters, position, camera_forward, avoidance_side)

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
                    rotation, omega, expected_delta_velocity / control_dt, control_dt,
                    avoidance_velocity)
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

            # Fixed-rate CSV telemetry, rather than flooding a file every
            # physics step.  Default: 10 samples per simulated second.
            if step % TELEMETRY_EVERY == 0:
                euler = p.getEulerFromQuaternion(quaternion)
                if writer:
                    writer.writerow([sim_time, waypoint_index + 1, *position, *velocity,
                        *controller.acceleration, *euler, *actual_thrusts, *weather_force,
                        actual_thrusts.sum(), np.linalg.norm(weather_force),
                        *(MASS * controller.disturbance_acceleration),
                        camera_obstacle_distance, *avoidance_velocity])
                if not args.headless:
                    state = 'HOLD' if route_complete else f'Waypoint {waypoint_index + 1}/{len(WAYPOINTS)}'
                    obstacle_text = ('clear' if not obstacles_enabled
                                     else f'{camera_obstacle_distance:.1f} m')
                    label = (f'{state} | weather {"ON" if weather_enabled else "OFF"} | tilt {tilt:.1f} deg\n'
                             f'camera obstacle: {obstacle_text} | P: push | W: weather')
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
            if not args.headless and step % 40 == 0:
                follow_drone_camera(position, quaternion)
    except KeyboardInterrupt:
        pass
    finally:
        if video_writer is not None:
            video_writer.release()
            print(f'Saved MP4 recording: {args.record}')
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

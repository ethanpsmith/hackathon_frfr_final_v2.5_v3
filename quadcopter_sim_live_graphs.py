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



DT = 2 / 240                  # Physics step, seconds.
CONTROL_EVERY = 10            # Run the flight controller every 10 physics steps.
TELEMETRY_HZ = 10             # CSV samples per simulated second.
TELEMETRY_EVERY = max(1, round(1.0 / (TELEMETRY_HZ * DT)))
RECORD_FPS = 30
RECORD_WIDTH = 960
RECORD_HEIGHT = 540
RECORD_EVERY = max(1, round(1.0 / (RECORD_FPS * DT)))
LIVE_GRAPH_SECONDS = 10.0
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
AVOID_START_DISTANCE = 7.0    # Brake and sidestep before reaching a block.
DRONE_RADIUS = math.hypot(ROTOR_OFFSET, ROTOR_OFFSET) + ROTOR_RADIUS
AVOID_MARGIN = 0.30          # Extra clearance beyond the outer propeller tips.
AVOID_CLEARANCE = DRONE_RADIUS + AVOID_MARGIN
AVOID_RELEASE_MARGIN = 0.45  # Continue sidestepping until comfortably clear.
AVOID_SIDE_SPEED = 2.0       # m/s while the forward route is blocked.
CAMERA_OFFSET = np.array([ROTOR_OFFSET + 0.12, 0.0, 0.10])

WAYPOINTS = np.array([
    [0.0, 0.0, 1.5],
    [15.0, 15.0, 1.5],
    [15.0, 15.0, 0.25],
    [15.0, 15.0, 1.5],
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


def route_velocity(target, position):
    """Limit the waypoint request before adding any obstacle correction."""
    desired = 1.3 * (np.asarray(target) - position)
    desired[:2] = limit_length(desired[:2], MAX_XY_SPEED)
    desired[2] = np.clip(desired[2], -MAX_Z_SPEED, MAX_Z_SPEED)
    return desired


def commanded_velocity(target, position, avoidance_velocity):
    desired = route_velocity(target, position) + avoidance_velocity
    desired[:2] = limit_length(desired[:2], MAX_XY_SPEED)
    desired[2] = np.clip(desired[2], -MAX_Z_SPEED, MAX_Z_SPEED)
    return desired


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
        # A distant waypoint used to swamp the correction before saturation.
        desired_velocity = commanded_velocity(target, position, avoidance_velocity)

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



class FlightRecorder:
    """Burn live graphs into every frame BEFORE sending it to the MP4 encoder."""

    GRAPH_HEIGHT = 300
    VERSION = 'LIVE GRAPHS v3'

    def __init__(self, path):
        from collections import deque
        from pathlib import Path
        import shutil
        import subprocess
        import tempfile
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError as error:
            raise RuntimeError(
                'Recording graphs requires Pillow: python3 -m pip install pillow'
            ) from error
        self.Image, self.ImageDraw = Image, ImageDraw
        self.path = str(Path(path).expanduser().resolve())
        if Path(self.path).suffix.lower() != '.mp4':
            raise ValueError('--record must specify a .mp4 file')
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self.size = (RECORD_WIDTH, RECORD_HEIGHT + self.GRAPH_HEIGHT)
        self.history = deque()
        self.frames = 0
        self.writer = None
        self.process = None
        self.error_log = None
        self.closed = False
        self.fonts = {}
        for size in (12, 14, 17):
            for name in ('DejaVuSans.ttf',
                         '/System/Library/Fonts/Supplemental/Arial.ttf'):
                try:
                    self.fonts[size] = ImageFont.truetype(name, size)
                    break
                except OSError:
                    continue
            else:
                self.fonts[size] = ImageFont.load_default()
        try:
            import cv2
        except ImportError:
            cv2 = None
        self.cv2 = cv2
        if cv2 is not None:
            self.writer = cv2.VideoWriter(
                self.path, cv2.VideoWriter_fourcc(*'mp4v'), RECORD_FPS, self.size)
            if not self.writer.isOpened():
                self.writer.release()
                self.writer = None
        if self.writer is None:
            ffmpeg = shutil.which('ffmpeg')
            if not ffmpeg:
                raise RuntimeError(
                    'MP4 encoding requires OpenCV or FFmpeg. Run: '
                    'python3 -m pip install opencv-python pillow')
            self.error_log = tempfile.TemporaryFile()
            self.process = subprocess.Popen([
                ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
                '-f', 'rawvideo', '-pixel_format', 'rgb24',
                '-video_size', f'{self.size[0]}x{self.size[1]}',
                '-framerate', str(RECORD_FPS), '-i', 'pipe:0',
                '-an', '-c:v', 'libx264', '-preset', 'veryfast',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', self.path,
            ], stdin=subprocess.PIPE, stderr=self.error_log)
        print(f'{self.VERSION}: graphs burned into every frame; '
              f'{self.size[0]}x{self.size[1]}, {RECORD_FPS} FPS')
        print(f'Recording file: {self.path}')

    def compose(self, camera_rgb, sim_time):
        """Keep the full camera image and append an opaque graph strip."""
        camera_rgb = np.asarray(camera_rgb, dtype=np.uint8)
        expected_shape = (RECORD_HEIGHT, RECORD_WIDTH, 3)
        if camera_rgb.shape != expected_shape:
            raise ValueError(f'Camera frame shape {camera_rgb.shape}, expected {expected_shape}')
        canvas = self.Image.new('RGB', self.size, (17, 23, 34))
        canvas.paste(self.Image.fromarray(np.ascontiguousarray(camera_rgb)), (0, 0))
        draw = self.ImageDraw.Draw(canvas)
        now, thrusts, weather, estimated = self.history[-1]
        header = (f'{self.VERSION}   |   t = {sim_time:.2f} s   |   '
                  f'Total thrust: {thrusts.sum():.1f} N   |   '
                  f'Weather: {np.linalg.norm(weather):.2f} N')
        draw.text((12, RECORD_HEIGHT + 10), header,
                  font=self.fonts[17], fill=(242, 246, 252))
        titles = ['Rotor thrust (N)', 'Applied weather force (N)',
                  'Estimated disturbance (N)']
        colors = [(78, 183, 255), (114, 231, 148), (255, 164, 95), (234, 132, 239)]
        samples = list(self.history)
        start = max(0.0, sim_time - LIVE_GRAPH_SECONDS)
        end = max(LIVE_GRAPH_SECONDS, sim_time)
        for index, title in enumerate(titles):
            left = index * RECORD_WIDTH // 3
            right = (index + 1) * RECORD_WIDTH // 3
            draw.text((left + 12, RECORD_HEIGHT + 45), title,
                      font=self.fonts[14], fill=(237, 242, 252))
            legends = ('M0', 'M1', 'M2', 'M3') if index == 0 else ('Fx', 'Fy', 'Fz')
            for k, label in enumerate(legends):
                lx = left + 14 + k * 65
                draw.line((lx, RECORD_HEIGHT + 75, lx + 12, RECORD_HEIGHT + 75),
                          fill=colors[k], width=3)
                draw.text((lx + 17, RECORD_HEIGHT + 68), label,
                          font=self.fonts[12], fill=(221, 230, 242))
            values = np.asarray([sample[index + 1] for sample in samples])
            if index == 0:
                low, high = 0.0, max(20.0, float(np.max(values)) * 1.15)
            else:
                high = max(0.5, float(np.max(np.abs(values))) * 1.2)
                low = -high
            x0, x1 = left + 48, right - 16
            y0, y1 = RECORD_HEIGHT + 99, self.size[1] - 31
            for fraction in (0.0, 0.5, 1.0):
                y = round(y0 + fraction * (y1 - y0))
                value = high - fraction * (high - low)
                draw.line((x0, y, x1, y), fill=(58, 70, 89), width=1)
                draw.text((left + 3, y - 7), f'{value:.1f}',
                          font=self.fonts[12], fill=(165, 181, 203))
            draw.line((x0, y0, x0, y1, x1, y1), fill=(112, 132, 158), width=1)
            draw.text((x0, y1 + 8), f'{start:.1f}s',
                      font=self.fonts[12], fill=(183, 198, 217))
            draw.text((x1 - 43, y1 + 8), f'{end:.1f}s',
                      font=self.fonts[12], fill=(183, 198, 217))
            for k in range(values.shape[1]):
                points = [
                    (round(x0 + np.clip((sample[0] - start) / (end - start), 0, 1) * (x1 - x0)),
                     round(y1 - np.clip((value - low) / (high - low), 0, 1) * (y1 - y0)))
                    for sample, value in zip(samples, values[:, k])
                ]
                if len(points) > 1:
                    draw.line(points, fill=colors[k], width=2)
                px, py = points[-1]
                draw.ellipse((px - 2, py - 2, px + 2, py + 2), fill=colors[k])
        return np.asarray(canvas)

    def add_frame(self, camera_rgb, sim_time, thrusts, weather, estimated):
        if self.closed:
            raise RuntimeError('Cannot record after closing the MP4')
        vectors = [np.asarray(v, dtype=float).copy() for v in (thrusts, weather, estimated)]
        if [v.shape for v in vectors] != [(4,), (3,), (3,)]:
            raise ValueError('Recording requires four thrusts and two three-component forces')
        if not all(np.all(np.isfinite(v)) for v in vectors):
            raise ValueError('Nonfinite video telemetry')
        self.history.append((float(sim_time), *vectors))
        while self.history and self.history[0][0] < sim_time - LIVE_GRAPH_SECONDS:
            self.history.popleft()
        frame = self.compose(camera_rgb, sim_time)
        if frame.shape != (self.size[1], self.size[0], 3):
            raise RuntimeError('Video frame dimensions do not match the encoder')
        if self.writer is not None:
            self.writer.write(self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2BGR))
        else:
            self.process.stdin.write(np.ascontiguousarray(frame).tobytes())
        self.frames += 1

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.writer is not None:
            self.writer.release()
        if self.process is not None:
            try:
                self.process.stdin.close()
                code = self.process.wait()
                if code:
                    self.error_log.seek(0)
                    raise RuntimeError(self.error_log.read().decode('utf-8', errors='replace'))
            finally:
                self.error_log.close()
        if self.frames:
            print(f'Saved {self.path}: {self.frames} frames with {self.VERSION}.')


def test_recording(path):
    """Exercise the real graph compositor and MP4 encoder without PyBullet."""
    recorder = FlightRecorder(path)
    image = recorder.Image.new('RGB', (RECORD_WIDTH, RECORD_HEIGHT), (214, 225, 237))
    draw = recorder.ImageDraw.Draw(image)
    draw.text((40, 190), 'RECORDING TEST - SYNTHETIC DATA',
              font=recorder.fonts[17], fill=(24, 39, 63))
    draw.text((40, 230), 'This checks the live graphs and MP4 encoder; it is not a flight simulation.',
              font=recorder.fonts[14], fill=(24, 39, 63))
    camera = np.asarray(image)
    try:
        for i in range(3 * RECORD_FPS):
            t = i / RECORD_FPS
            thrusts = 13.25 + 3 * np.sin(2 * t + np.arange(4) * 0.7)
            weather = np.array([np.sin(3*t), 0.7*np.cos(2*t), -0.2-0.1*np.sin(t)])
            estimated = 0.85 * weather
            recorder.add_frame(camera, t, thrusts, weather, estimated)
    finally:
        recorder.close()


def drone_camera_depth(drone, ground, position, rotation, route_direction):
    """Render the nose camera pointed along the current route direction."""
    # The camera stays mounted ahead of the propellers, but its gimbal turns
    # horizontally toward the waypoint.  Without this, the drone slides on a
    # diagonal route while the fixed camera stares along world +X.
    eye = position + rotation @ CAMERA_OFFSET
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
    # axial depth along camera forward (not Euclidean ray distance).
    depth_meters = CAMERA_FAR * CAMERA_NEAR / (
        CAMERA_FAR - (CAMERA_FAR - CAMERA_NEAR) * depth_buffer
    )
    return depth_meters, forward


def camera_avoidance_velocity(depth_meters, position, camera_forward, locked_side,
                              camera_offset=None, nominal_velocity=None):
    """Brake and sidestep when camera points intersect an inflated flight corridor.

    Uses only the masked depth image, not the known obstacle layout. Ground
    and self are already excluded by object ID, including during takeoff.
    """
    depth = np.asarray(depth_meters, dtype=float)
    h, w = depth.shape
    camera_left = np.cross([0.0, 0.0, 1.0], camera_forward)
    camera_left /= max(np.linalg.norm(camera_left), 1e-12)
    offset = CAMERA_OFFSET if camera_offset is None else np.asarray(camera_offset)
    # Reconstruct level-camera points relative to the drone centre. The
    # footprint includes every propeller, plus a clearance margin. A half
    # pixel allowance accounts for the finite camera angular resolution.
    tangent = math.tan(math.radians(CAMERA_FOV) / 2)
    rows, columns = np.indices((h, w))
    ahead = depth + float(offset @ camera_forward)
    lateral = (1 - 2 * (columns + 0.5) / w) * depth * tangent * w / h
    lateral += float(offset @ camera_left)
    vertical = (1 - 2 * (rows + 0.5) / h) * depth * tangent + offset[2]
    pixel_margin = depth * tangent / h
    valid = np.isfinite(depth) & (depth > CAMERA_NEAR) & (depth < CAMERA_FAR - 0.01)
    radius = AVOID_CLEARANCE + (AVOID_RELEASE_MARGIN if locked_side else 0.0)
    nearby = valid & (ahead > -DRONE_RADIUS) & (ahead < AVOID_START_DISTANCE)
    # A conservative sphere also covers propeller height when the drone tilts.
    nearby &= np.abs(vertical) < AVOID_CLEARANCE + pixel_margin
    corridor = nearby & (np.abs(lateral) < radius + pixel_margin)
    if not np.any(corridor):
        return np.zeros(3), float('inf'), 0.0
    # Minimum over actual object pixels: a narrow box must not be discarded
    # merely because sky occupies more than 95 percent of the image.
    front_distance = float(np.min(ahead[corridor]))
    if locked_side:
        side = locked_side
    else:
        left_obstacles = nearby & (lateral >= 0) & (lateral < 2 * radius)
        right_obstacles = nearby & (lateral < 0) & (lateral > -2 * radius)
        left_distance = float(np.min(ahead[left_obstacles], initial=CAMERA_FAR))
        right_distance = float(np.min(ahead[right_obstacles], initial=CAMERA_FAR))
        side = 1.0 if left_distance > right_distance else -1.0
    nominal = (MAX_XY_SPEED * camera_forward if nominal_velocity is None
               else np.asarray(nominal_velocity))
    # Cancel the bounded horizontal route request completely. Forward travel
    # resumes only after the propeller-sized corridor is clear. Altitude hold
    # still comes from the normal controller.
    avoidance_velocity = side * AVOID_SIDE_SPEED * camera_left
    avoidance_velocity[:2] -= nominal[:2]
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
    arm_collision = p.createCollisionShape(p.GEOM_BOX,
        halfExtents=[math.sqrt(2) * ROTOR_OFFSET, 0.018, 0.012])
    rotor_collision = p.createCollisionShape(p.GEOM_CYLINDER,
        radius=ROTOR_RADIUS, height=0.018)
    # Front rotors blue, rear rotors orange to make attitude easy to see.
    front = p.createVisualShape(p.GEOM_CYLINDER, radius=ROTOR_RADIUS,
                                length=0.018, rgbaColor=[0.10, 0.60, 0.95, 1])
    rear = p.createVisualShape(p.GEOM_CYLINDER, radius=ROTOR_RADIUS,
                               length=0.018, rgbaColor=[1.0, 0.48, 0.08, 1])
    identity = [0., 0., 0., 1.]
    drone = p.createMultiBody(
        baseMass=MASS, baseCollisionShapeIndex=collision, baseVisualShapeIndex=body,
        basePosition=START_POSITION,
        linkMasses=[0.] * 6,
        linkCollisionShapeIndices=[arm_collision] * 2 + [rotor_collision] * 4,
        linkVisualShapeIndices=[arm, arm, front, front, rear, rear],
        linkPositions=[[0., 0., 0.], [0., 0., 0.]] + ROTOR_POSITIONS.tolist(),
        linkOrientations=[p.getQuaternionFromEuler([0, 0, math.pi / 4]),
                          p.getQuaternionFromEuler([0, 0, -math.pi / 4])] + [identity] * 4,
        linkInertialFramePositions=[[0., 0., 0.]] * 6,
        linkInertialFrameOrientations=[identity] * 6,
        linkParentIndices=[0] * 6, linkJointTypes=[p.JOINT_FIXED] * 6,
        linkJointAxis=[[0., 0., 1.]] * 6)
    # Model the entire mass distribution in the base inertia, rather than using
    # just the small centre box's inertia. Fixed links also model collisions.
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
                        help='Record a 30 FPS chase-camera MP4 with live graphs below the flight view.')
    parser.add_argument('--record-test', action='store_true',
                        help='With --record, encode a 3-second synthetic graph test without PyBullet.')
    parser.add_argument('--version', action='version', version=FlightRecorder.VERSION)
    args = parser.parse_args()
    if args.record_test:
        if not args.record:
            parser.error('--record-test requires --record FILE.mp4')
        test_recording(args.record)
        return
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
    collided_obstacles = set()
    stream = open(args.csv, 'w', newline='') if args.csv else None
    writer = csv.writer(stream) if stream else None
    recorder = None
    if args.record:
        try:
            recorder = FlightRecorder(args.record)
        except Exception:
            if stream:
                stream.close()
            p.disconnect(client)
            raise
    if writer:
        writer.writerow(['time_s', 'waypoint', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                         'ax', 'ay', 'az', 'roll_rad', 'pitch_rad', 'yaw_rad',
                         'T0_N', 'T1_N', 'T2_N', 'T3_N', 'weather_Fx', 'weather_Fy',
                         'weather_Fz', 'total_thrust_N', 'weather_force_N',
                         'estimated_Fx', 'estimated_Fy', 'estimated_Fz',
                         'camera_obstacle_m', 'avoid_vx', 'avoid_vy', 'avoid_vz',
                         'obstacle_collision'])
    print('Four-rotor simulation. P: sideways push. W: toggle weather. Ctrl+C: exit.')
    if obstacles_enabled:
        print(f'Camera avoidance enabled for {len(obstacle_ids)} obstacles. '
              'Use --no-obstacles for the baseline route.')
        print(f'Propeller radius envelope: {DRONE_RADIUS:.2f} m; '
              f'avoidance clearance: {AVOID_CLEARANCE:.2f} m; '
              f'braking trigger: {AVOID_START_DISTANCE:.1f} m.')

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
                    depth_meters, position, camera_forward, avoidance_side,
                    rotation @ CAMERA_OFFSET,
                    route_velocity(WAYPOINTS[waypoint_index], position))

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

            # Every recorded frame receives current telemetry and the live
            # graph strip BEFORE encoding. CSV output is optional and separate.
            if recorder is not None and step % RECORD_EVERY == 0:
                recorder.add_frame(
                    render_chase_frame(position, rotation), sim_time,
                    actual_thrusts, weather_force,
                    MASS * controller.disturbance_acceleration)

            # Fixed-rate CSV telemetry, rather than flooding a file every
            # physics step.  Default: 10 samples per simulated second.
            if step % TELEMETRY_EVERY == 0:
                euler = p.getEulerFromQuaternion(quaternion)
                if writer:
                    writer.writerow([sim_time, waypoint_index + 1, *position, *velocity,
                        *controller.acceleration, *euler, *actual_thrusts, *weather_force,
                        actual_thrusts.sum(), np.linalg.norm(weather_force),
                        *(MASS * controller.disturbance_acceleration),
                        camera_obstacle_distance, *avoidance_velocity,
                        bool(collided_obstacles)])
                if not args.headless:
                    state = 'HOLD' if route_complete else f'Waypoint {waypoint_index + 1}/{len(WAYPOINTS)}'
                    obstacle_text = ('clear' if not obstacles_enabled
                                     else f'{camera_obstacle_distance:.1f} m')
                    label = (f'{state} | weather {"ON" if weather_enabled else "OFF"} | tilt {tilt:.1f} deg\n'
                             f'camera obstacle: {obstacle_text} | P: push | W: weather')
                    status_id = p.addUserDebugText(label, [0, 0, 2.8], [0.1, 0.1, 0.1],
                                                   textSize=1.0, replaceItemUniqueId=status_id)

            p.stepSimulation()
            # Check all body/link contacts, including the rotor disks. This is
            # diagnostic only; avoidance never reads known obstacle positions.
            for contact in p.getContactPoints(bodyA=drone):
                hit = contact[2]
                if hit in obstacle_ids and hit not in collided_obstacles:
                    collided_obstacles.add(hit)
                    print(f'{(step + 1) * DT:6.2f}s: OBSTACLE COLLISION '
                          f'(box {obstacle_ids.index(hit) + 1}, drone link {contact[3]})')
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
        if recorder is not None:
            recorder.close()
        if stream:
            stream.close()
        if p.isConnected(client):
            final_position, _ = p.getBasePositionAndOrientation(drone)
            error = np.linalg.norm(np.array(final_position) - WAYPOINTS[waypoint_index])
            print(f'Final error to active target: {error:.3f} m | peak tilt: {peak_tilt:.1f} deg | '
                  f'min height: {minimum_height:.3f} m | route complete: {route_complete} | '
                  f'obstacle collision: {bool(collided_obstacles)}')
            p.disconnect(client)


if __name__ == '__main__':
    main()

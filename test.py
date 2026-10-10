import time
import pybullet as p
import pybullet_data

block_mass = 1 #kg
dt = 0.1/240

client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)



plane_id = p.loadURDF("plane.urdf")
stl_path="/Users/ethansmith/img63/quad_delivery_vtol.step"

# Physics shape: simple box around the drone
collision_shape = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.25, 0.25, 0.08],  # adjust to fit the drone, in meters
)

# Appearance: load the STL mesh
visual_shape = p.createVisualShape(
    p.GEOM_MESH,
    fileName=stl_path,
    meshScale=[0.001, 0.001, 0.001],  # use if the STL dimensions are in millimeters
    rgbaColor=[0.2, 0.5, 0.9, 1],
)

box_id_1 = p.createMultiBody(
    baseMass=block_mass,
    baseCollisionShapeIndex=collision_shape,
    baseVisualShapeIndex=visual_shape,
    basePosition=[0, 0, 0],
)

kp_xy = 2
kd_xy = 1
kp = 2
kd = 1
hor_speed = 5.0
ver_speed = 1.0

p.changeDynamics(box_id_1, -1, mass=block_mass) # adding thrust
p.resetBaseVelocity(box_id_1, linearVelocity=[0, 0, 0])




waypoint_list = [
    [0,0,1.5],
    [1,1,1.5],
    [1,1,0.5],
    [1,1,1.5],
    [0,0,1.5],
    [0,0,0.5]
]

waypoint_index = 0
arrival_radius = 0.5


def apply_force(box_id, target, kp_xy=5, kd_xy=3, kp_z=2, kd_z=1):
    position, orientation = p.getBasePositionAndOrientation(box_id)
    linear_velocity, angular_velocity = p.getBaseVelocity(box_id)

    # Vertical force: counter gravity, then correct altitude error
    height_error = target[2] - position[2]
    force_up = block_mass * 9.81 + kp_z * height_error - kd_z * linear_velocity[2]
    force_up = max(0, force_up)

    # Horizontal forces: move toward target and damp motion
    error_x = target[0] - position[0]
    error_y = target[1] - position[1]
    force_x = kp_xy * error_x - kd_xy * linear_velocity[0]
    force_y = kp_xy * error_y - kd_xy * linear_velocity[1]

    p.applyExternalForce(
        objectUniqueId=box_id,
        linkIndex=-1,
        forceObj=[force_x, force_y, force_up],
        posObj=position,
        flags=p.WORLD_FRAME,
    )
    #print (net_force, position[0], position[1],position[2])


#box_id_2 = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=box_1, basePosition=[0,0.5,4])
# Advance physics and keep the window open
while p.isConnected():
    if waypoint_index >= len(waypoint_list):
        print("Route complete")
        break

    target = waypoint_list[waypoint_index]


    target = waypoint_list[waypoint_index]


    position, orientation = p.getBasePositionAndOrientation(box_id_1)

    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dz = target[2] - position[2]

    distance = (dx**2 + dy**2 + dz**2) ** 0.5
    print(
        f"waypoint {waypoint_index + 1}/{len(waypoint_list)} | "
        f"position=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) | "
        f"distance={distance:.2f} m"
    )

    if distance < arrival_radius:
        print(f"reached waypoint {waypoint_index}: {target}")
        waypoint_index+=1
        continue



    apply_force(box_id_1,target)



    p.stepSimulation()
    time.sleep(dt)

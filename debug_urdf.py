import mujoco
import numpy as np

# Load the URDF
model = mujoco.MjModel.from_xml_path("robotDescription/robot.urdf")
data = mujoco.MjData(model)

# Print model information
print("=== Model Information ===")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Gravity: {model.opt.gravity}")

print("\n=== Bodies ===")
for i in range(model.nbody):
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    print(f"Body {i}: {body_name}")

print("\n=== Joints ===")
for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    joint_type = model.jnt_type[i]
    joint_axis = model.jnt_axis[i]
    joint_range = model.jnt_range[i]

    type_names = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    print(f"Joint {i}: {joint_name}")
    print(f"  Type: {type_names.get(joint_type, 'unknown')}")
    print(f"  Axis: {joint_axis}")
    print(f"  Range: {joint_range}")
    print(
        f"  Damping: {model.dof_damping[i] if i < len(model.dof_damping) else 'N/A'}")
    print(
        f"  Friction: {model.dof_frictionloss[i] if i < len(model.dof_frictionloss) else 'N/A'}")

# Run a short simulation to see if joint moves
print("\n=== Testing slider movement ===")
print(f"Initial joint position: {data.qpos}")

# Step the simulation
for _ in range(100):
    mujoco.mj_step(model, data)

print(f"After 100 steps: {data.qpos}")
print(f"Joint velocity: {data.qvel}")

if np.any(np.abs(data.qvel) > 0.001):
    print("✓ Joint is moving!")
else:
    print("✗ Joint is NOT moving - may need adjustment")

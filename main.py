import mujoco
import mujoco.viewer
import numpy as np

# Load the model
print("Loading model...")
model = mujoco.MjModel.from_xml_path(
    '/Users/shivamwalia/Desktop/mujocoProject/mujocoRobotDescription/scene.xml')
data = mujoco.MjData(model)

print("Model loaded successfully!")
print(f"Number of bodies: {model.nbody}")
print(f"Number of geoms: {model.ngeom}")
print(f"Number of joints: {model.njnt}")

# Print collision info
print("\n" + "="*60)
print("COLLISION GEOMETRY INFO")
print("="*60)
for i in range(model.ngeom):
    geom_name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom_{i}"
    contype = model.geom_contype[i]
    conaffinity = model.geom_conaffinity[i]
    geom_type = model.geom_type[i]
    type_names = ["plane", "hfield", "sphere", "capsule",
                  "ellipsoid", "cylinder", "box", "mesh"]
    type_name = type_names[geom_type] if geom_type < len(
        type_names) else "unknown"

    if contype > 0 or conaffinity > 0:  # Only show collision geoms
        print(
            f"{geom_name:30s} type={type_name:10s} contype={contype:3d} conaffinity={conaffinity:3d}")

print("\n" + "="*60)
print("LAUNCHING VIEWER")
print("="*60)
print("Controls:")
print("  - Double-click and drag to rotate view")
print("  - Right-click and drag to pan")
print("  - Scroll to zoom")
print("  - Ctrl+Right-click to apply forces to bodies")
print("  - Press SPACE to pause/unpause")
print("  - Press ESC to exit")
print("="*60)

# Launch viewer
mujoco.viewer.launch(model, data)

print("\nSimulation ended.")

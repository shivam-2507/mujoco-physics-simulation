import mujoco
import mujoco.viewer
import numpy as np
import sys
import threading
import time


def load_urdf_in_mujoco(urdf_path):
    """
    Load a URDF file in MuJoCo and launch the interactive viewer.

    Args:
        urdf_path (str): Path to the URDF file
    """
    try:
        # Load the URDF file - MuJoCo automatically converts it to MJCF format
        model = mujoco.MjModel.from_xml_path(urdf_path)

        # Create data structure for simulation
        data = mujoco.MjData(model)

        # Set initial position of slider to start at top of ramp
        # Negative value moves it backward (up the ramp)
        if model.njnt > 0:
            data.qpos[0] = -0.4  # Start 400mm up the ramp (adjust this value)

        # Launch the interactive viewer
        print(f"Successfully loaded URDF: {urdf_path}")
        print(f"Model has {model.nbody} bodies and {model.njnt} joints")

        # Debug joint axis and gravity
        if model.njnt > 0:
            joint_axis = model.jnt_axis[0]
            print(f"Joint axis: {joint_axis}")
            print(f"Gravity: {model.opt.gravity}")
            print(f"Joint damping: {model.dof_damping[0]}")
            print(f"Joint friction: {model.dof_frictionloss[0]}")
            print(f"Starting position: {data.qpos[0]}")

        # Check if running on macOS and provide instructions
        if sys.platform == "darwin":
            print("\nOn macOS, use 'mjpython' instead of 'python3' to run this script:")
            print(f"  mjpython {__file__}")
            print("\nAlternatively, launching viewer with launch() instead...")

        print("\n=== Real-time velocity monitoring ===")
        print("Position (m) | Velocity (m/s) | Time (s)")
        print("-" * 50)

        # Track if viewer is still running
        viewer_running = [True]

        # Function to apply gravity force along the ramp
        # Force = mass * g * sin(30°) = 5.0 * 9.81 * 0.5 = 24.525 N
        def apply_ramp_gravity(model, data):
            if model.njnt > 0:
                data.qfrc_applied[0] = 24.525  # Constant force down the ramp

        # Function to print velocity periodically
        def monitor_velocity():
            last_time = 0.0
            while viewer_running[0]:
                # Apply gravity force at every step
                apply_ramp_gravity(model, data)

                if model.njnt > 0 and data.time - last_time >= 0.1:
                    pos = data.qpos[0]
                    vel = data.qvel[0]
                    print(f"{pos:11.4f} | {vel:14.4f} | {data.time:8.3f}")
                    last_time = data.time
                time.sleep(0.05)

        # Start monitoring thread
        monitor_thread = threading.Thread(target=monitor_velocity, daemon=True)
        monitor_thread.start()

        # Use launch() which works with regular Python on macOS
        mujoco.viewer.launch(model, data)

        viewer_running[0] = False

    except Exception as e:
        print(f"Error loading URDF: {e}")
        print("\nMake sure:")
        print("1. The URDF file path is correct")
        print("2. All mesh files referenced in the URDF are accessible")
        print("3. The URDF is valid")

        if sys.platform == "darwin" and "mjpython" in str(e):
            print("\nFor better viewer experience on macOS, run with:")
            print(f"  mjpython {__file__}")


if __name__ == "__main__":
    # Use relative path from project root
    # This ensures MuJoCo can resolve package:// paths correctly
    urdf_file = "robotDescription/robot.urdf"

    # Alternative: use absolute path if needed
    # urdf_file = "/Users/shivamwalia/Desktop/mujocoProject/robotDescription/robot.urdf"

    load_urdf_in_mujoco(urdf_file)

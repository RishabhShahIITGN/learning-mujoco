import mujoco
import mujoco.viewer
import time
import os
import math  # <--- ADDED THIS IMPORT

# 1. Get the directory where THIS python file is located
current_dir = os.path.dirname(os.path.abspath(__file__))

# 2. Join that directory with the path to the robot
xml_path = os.path.join(current_dir, "mujoco_menagerie/universal_robots_ur5e/scene.xml")

# Check if file exists
if not os.path.exists(xml_path):
    print(f"Error: Could not find the robot file at: {xml_path}")
    exit()

# Load the Model
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Loaded UR5e successfully!")

    print("Joint names:")

    for i in range(model.nu): # model.nu = number of actuators/controls
        # This is a bit advanced way to get names, just trust me for now
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"Index {i}: {name}")

    start_time = time.time()
    while viewer.is_running():
        # Make the robot wave
        now = time.time() - start_time
        
        # --- FIXED LINES BELOW ---
        data.ctrl[0] = 0.5 * math.sin(now)  # Rotate base
        data.ctrl[4] = 1.0 * math.cos(now)  # Wrist move

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)
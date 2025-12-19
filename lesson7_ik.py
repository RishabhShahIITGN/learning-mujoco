import mujoco
import mujoco.viewer
import time
import math
import os

# 1. Get the directory where THIS python file is located
current_dir = os.path.dirname(os.path.abspath(__file__))

# 2. UPDATE THE PATH: Point to the new location inside the menagerie folder
# Note: We go into mujoco_menagerie -> universal_robots_ur5e -> lesson7_ik.xml
xml_path = os.path.join(current_dir, "mujoco_menagerie/universal_robots_ur5e/lesson7_ik.xml")

# Check if file exists
if not os.path.exists(xml_path):
    print(f"Error: Could not find the XML file at: {xml_path}")
    exit()

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Get the ID of the mocap body
target_id = model.body("target").mocapid[0]

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("IK Simulation Started! Watch the Karate Chop.")
    
    start_time = time.time()
    while viewer.is_running():
        time_now = time.time() - start_time
        
        # DEFINING A SWEEP MOTION
        # We want to sweep from Y = +0.5 to Y = -0.5
        # We use cos(time) to go back and forth
        
        sweep_y = 0.5 * math.cos(time_now)
        
        # Fixed X distance (Forward reach)
        x = 0.5
        
        # LOW HEIGHT (To hit the box)
        # We set Z to 0.1 so the hand scrapes just above the table
        z = 0.1
        
        # Update the ghost target position
        data.mocap_pos[target_id] = [x, sweep_y, z]
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)
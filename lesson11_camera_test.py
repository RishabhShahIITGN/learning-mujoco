import mujoco
import numpy as np
import imageio
from ur5_env import UR5ReachEnv

# 1. Start the Simulation
# We don't need "render_mode=human" because we are grabbing pixels directly
env = UR5ReachEnv()
env.reset()

# 2. Setup the Renderer
# This tool converts the physics data into pixels
# We explicitly tell it to use the camera
renderer = mujoco.Renderer(env.model, height=480, width=640)

print("Moving robot to look at the ball...")

# 3. Move the Robot to look DOWN at the ball
# We change the joints to create a "looking down" pose
# [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
# Wrist2 (index 4) controls the up/down tilt of the camera
action = np.array([0, -1.57, 1.57, -1.57, -1.57, 0], dtype=np.float32)

env.data.qpos[:] = action 
mujoco.mj_step(env.model, env.data)

# 4. Snap a Picture
renderer.update_scene(env.data, camera="eye_in_hand")
pixels = renderer.render()

# 5. Save it
imageio.imwrite("robot_view.png", pixels)

print("Success! Check 'robot_view.png' to see what the robot sees.")
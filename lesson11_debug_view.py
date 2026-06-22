import mujoco
import numpy as np
import imageio
from ur5_env import UR5ReachEnv

# 1. Start Environment
env = UR5ReachEnv()
env.reset()

print("DEBUG MODE: Teleporting ball to face...")

# 2. Force Robot Pose (Looking forward/down)
# [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
pose = np.array([0, -1.57, 1.57, -1.57, -1.57, 0], dtype=np.float32)
env.data.qpos[:] = pose

# 3. Force Ball Position (Directly in front of the hand)
# The hand is roughly at (0, 0.4, 0.6) in this pose
# We place the ball slightly in front of it (Y+ direction)
target_id = env.model.body("target").mocapid[0]
env.data.mocap_pos[target_id] = [0.1, 0.6, 0.5] # X, Y, Z

# 4. Update Physics
mujoco.mj_step(env.model, env.data)

# 5. Snap Picture
renderer = mujoco.Renderer(env.model, height=480, width=640)
renderer.update_scene(env.data, camera="eye_in_hand")
pixels = renderer.render()

imageio.imwrite("robot_view_debug.png", pixels)
print("Saved 'robot_view_debug.png'. You MUST see the green ball now!")
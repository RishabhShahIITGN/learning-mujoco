import mujoco
import numpy as np
import imageio
from ur5_env import UR5ReachEnv
from stable_baselines3 import PPO

# 1. Setup Environment
env = UR5ReachEnv()
obs, _ = env.reset()

# 2. Load the Trained Brain
model = PPO.load("ur5_reach_model", env=env)
print("Brain loaded! Moving robot to the ball...")

# 3. Let the AI drive for 60 steps
# This moves the hand (and the camera) right up to the target
# ... inside the loop ...
for i in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, _, terminated, truncated, _ = env.step(action)
    
    # Calculate distance for debugging
    # The last 3 numbers of obs are the relative vector (Distance X, Y, Z)
    distance = np.linalg.norm(obs[-3:])
    print(f"Step {i}: Distance to ball = {distance:.3f} m")

    if terminated:
        print("TARGET HIT! Taking picture...")
        break

# 4. Snap the Picture
print("Taking photo now...")
renderer = mujoco.Renderer(env.model, height=480, width=640)
renderer.update_scene(env.data, camera="eye_in_hand")
pixels = renderer.render()

imageio.imwrite("robot_view_smart.png", pixels)
print("Success! Open 'robot_view_smart.png'")
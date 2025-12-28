import gymnasium as gym
from stable_baselines3 import PPO
import imageio
import numpy as np
import mujoco
from ur5_env import UR5ReachEnv

# 1. Load the Environment (No window needed)
env = UR5ReachEnv(render_mode="rgb_array")

# 2. Load your trained brain
try:
    model = PPO.load("ur5_reach_model", env=env)
    print("Model loaded successfully!")
except:
    print("Error: Could not find 'ur5_reach_model.zip'.")
    exit()

# 3. Setup the Camera Renderer
# We attach a renderer to the physics model
renderer = mujoco.Renderer(env.model, height=480, width=640)

print("------------------------------------------------")
print("Recording video... (This takes about 10 seconds)")
print("------------------------------------------------")

frames = []
obs, _ = env.reset()

# Record for 300 frames (approx 10-15 seconds of video)
for i in range(300):
    # Get the AI's action
    action, _ = model.predict(obs, deterministic=True)
    
    # Step the physics
    obs, _, terminated, truncated, _ = env.step(action)
    
    # Render the current frame
    renderer.update_scene(env.data)
    pixels = renderer.render()
    frames.append(pixels)

    # Reset if the robot wins or times out
    if terminated or truncated:
        obs, _ = env.reset()

# 4. Save to MP4
video_path = "ur5_victory.mp4"
imageio.mimsave(video_path, frames, fps=30)

print(f"------------------------------------------------")
print(f"DONE! Video saved as '{video_path}'")
print(f"------------------------------------------------")
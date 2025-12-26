from ur5_env import UR5ReachEnv
from stable_baselines3 import PPO
import os

# 1. Instantiate Environment
env = UR5ReachEnv(render_mode="human")

# 2. Create Model
model = PPO("MlpPolicy", env, verbose=1)

print("------------------------------------------------")
print("STARTING UR5e TRAINING (With Relative Coordinates)")
print("------------------------------------------------")

# 3. Train
# With relative coordinates, 100k steps should be enough to see hitting!
model.learn(total_timesteps=100000)

print("------------------------------------------------")
print("TRAINING DONE! SAVING MODEL...")
print("------------------------------------------------")

# 4. SAVE THE MODEL (Crucial Step!)
model.save("ur5_reach_model")

print("Model saved as 'ur5_reach_model.zip'. Now you can run the test script.")
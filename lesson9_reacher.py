import gymnasium as gym
from stable_baselines3 import PPO
import os

# 1. CREATE THE ENVIRONMENT
# "Reacher-v4" is a 2-joint robot arm trying to touch a random target.
env = gym.make("Reacher-v4", render_mode="human")

# 2. DEFINE THE BRAIN
# We use the same PPO brain as before. It's very versatile.
model = PPO("MlpPolicy", env, verbose=1)

print("---------------------------------------")
print("STARTING TRAINING... (Arm will flail wildly)")
print("---------------------------------------")

# 3. TRAIN
# Reaching is harder than balancing! 
# It usually needs 100k+ steps to get perfect, but 
# we will do 20k just to see it start learning.
model.learn(total_timesteps=200000)

print("---------------------------------------")
print("TRAINING FINISHED! Saving model...")
print("---------------------------------------")

# Save the brain
model.save("my_reacher_robot")

# 4. TEST
obs, info = env.reset()

while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, dones, truncated, info = env.step(action)
    
    # If the time runs out, reset
    if truncated or dones:
        obs, info = env.reset()
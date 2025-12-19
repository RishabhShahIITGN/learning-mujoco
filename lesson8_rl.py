import gymnasium as gym
from stable_baselines3 import PPO
import os

# 1. CREATE THE ENVIRONMENT
# We use the standard MuJoCo Inverted Pendulum task.
# render_mode="human" allows us to see it learning in real-time.
env = gym.make("InvertedPendulum-v5", render_mode="human")
print("Environment Created!")

# 2. DEFINE THE BRAIN (The Agent)
# "MlpPolicy" = Multi-Layer Perceptron (A standard Neural Network)
# verbose=1 = Print progress to the terminal
model = PPO("MlpPolicy", env, verbose=1)
print("Brain Created!")

print("---------------------------------------")
print("STARTING TRAINING... (Watch the Stick fail, then learn!)")
print("---------------------------------------")

# 3. TRAIN THE BRAIN
# We give it 50,000 steps to practice. 
# It will try random moves, get a score, and improve.
model.learn(total_timesteps=50000)

print("---------------------------------------")
print("TRAINING FINISHED! Now showing off...")
print("---------------------------------------")

# 4. TEST THE TRAINED BRAIN
# Reset the world
obs, info = env.reset()
# ... training finishes ...

# 5. SAVE THE TRAINED MODEL
# This creates a file named "my_balanced_stick.zip" in your folder
model.save("my_balanced_stick")
print("Model saved successfully!")

# ... testing loop ...

# Loop forever to watch the result
while True:
    # Ask the AI: "Based on what you see (obs), what should I do?"
    action, _states = model.predict(obs, deterministic=True)
    
    # Take that action
    obs, rewards, dones, truncated, info = env.step(action)
    
    # If the stick falls over, reset
    if dones or truncated:
        obs, info = env.reset()
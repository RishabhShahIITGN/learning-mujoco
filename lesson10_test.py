from ur5_env import UR5ReachEnv
from stable_baselines3 import PPO

env = UR5ReachEnv(render_mode="human")

# Assuming your script saved the model as "ur5_reach_model" or similar
# If you didn't save explicitly, check for a .zip file in the folder
try:
    model = PPO.load("ur5_reach_model", env=env) # Change name if needed
except:
    print("Model not found! Did you save it?")
    exit()

obs, _ = env.reset()
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, _, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        obs, _ = env.reset()
import mujoco
from ur5_env import UR5ReachEnv

env = UR5ReachEnv()
print("---------------------------------------------")
# Access the 'target' body, then its first geom (the sphere)
target_body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "target")
geom_id = env.model.body_geomadr[target_body_id]
rgba = env.model.geom_rgba[geom_id]

print(f"Current Ball Color: {rgba}")
print("---------------------------------------------")

if rgba[3] == 1.0:
    print("SUCCESS: The ball is SOLID.")
else:
    print("FAILURE: The ball is TRANSPARENT (Ghost).")
    print("You need to edit 'lesson10_ur5_rl.xml'.")
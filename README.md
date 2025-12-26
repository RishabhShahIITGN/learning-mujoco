# MuJoCo Robotics Learning Journey

This repository documents my progression in robotic simulation using **MuJoCo** and **Python**. It covers fundamental concepts of rigid body dynamics, control theory, inverse kinematics, and **Deep Reinforcement Learning**, culminating in a self-learning **UR5e industrial manipulator**.

## 🛠️ Tech Stack
* **Physics Engine:** MuJoCo
* **Language:** Python 3
* **Libraries:** `mujoco`, `numpy`, `gymnasium`, `stable-baselines3`, `imageio`
* **Robots:** Universal Robots UR5e (via MuJoCo Menagerie)

## 📂 Project Structure

### Basics & Physics
| File | Description |
| :--- | :--- |
| `lesson1_basics.py` | Introduction to XML (MJCF) bodies and geoms. |
| `lesson2_pendulum.py` | Simulating a double pendulum dynamics. |
| `lesson3_motor.py` | Actuation and joint control torque. |
| `lesson4_sensors.py` | Reading sensor data (qpos, qvel) and equilibrium states. |
| `lesson5_friction.py` | Contact physics, friction parameters, and stability. |

### Robotics Control
| File | Description |
| :--- | :--- |
| `lesson6_ur5.py` | Loading the **UR5e** mesh model from DeepMind's Menagerie. |
| `lesson7_ik.py` | **Inverse Kinematics** implementation using Mocap bodies to make the arm track a target. |

### Reinforcement Learning (AI)
| File | Description |
| :--- | :--- |
| `lesson8_rl.py` | **RL Hello World**: Training a PPO agent to balance an Inverted Pendulum. |
| `lesson9_reacher.py` | **Kinematic Learning**: Training a 2-joint arm to reach random targets (Reacher-v4). |
| `lesson10_train.py` | **The Boss Fight**: Custom Gym Environment for a **6-DOF UR5e**. Used **Relative Coordinates** and **PPO** to train the robot to reach a target ball from scratch. |
| `ur5_env.py` | **Custom Gym Wrapper**: The bridge between MuJoCo physics and the AI brain. Handles reward logic (distance minimization) and state observation. |

## 🚀 How to Run the Final AI
1. Clone the repository:
   ```bash
   git clone [https://github.com/RishabhShahIITGN/learning-mujoco.git](https://github.com/RishabhShahIITGN/learning-mujoco.git)
   cd learning-mujoco
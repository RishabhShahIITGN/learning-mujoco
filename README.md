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
| `lesson1_basics.py` | The Basic Simulation Loop Introduces the core MuJoCo physics loop by simulating a falling sphere. It demonstrates how to compile an XML model, advance physics time (mj_step), and update the 3D graphics window (viewer.sync). |
| `lesson2_pendulum.py` | A simulation of double pendulum dynamics demonstrating hinge joints in MuJoCo. It applies an initial velocity (qvel) to the top joint to showcase complex, chaotic physical motion. |
| `lesson3_motor.py` | Motor Control Introduces actuators by attaching a motor to the pendulum's shoulder joint. It demonstrates sending continuous torque commands using a mathematical sine wave to actively drive the robot's motion. |
| `lesson4_sensors.py` | Applies a constant torque to a double pendulum, demonstrating how the unmotorized lower arm moves purely via physics and gravity. It also shows how to extract real-time angles (qpos) and velocities (qvel) directly from MuJoCo's internal engine state without building simulated hardware sensors. |
| `lesson5_friction.py` | Demonstrates unconstrained 6-DOF (Degree of Freedom) movement using a free joint. Applies an initial linear velocity to a box, showcasing how MuJoCo's ground-plane sliding friction naturally decelerates the object over time. |

### Robotics Control
| File | Description |
| :--- | :--- |
| `lesson6_ur5.py` | Loads the UR5e industrial robot from an external XML file to programmatically explore its joints and attachment sites. Applies basic mathematical signals to tinker with specific joints and understand its motion. |
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

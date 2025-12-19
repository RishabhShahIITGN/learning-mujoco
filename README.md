# MuJoCo Robotics Learning Journey

This repository documents my progression in robotic simulation using **MuJoCo** and **Python**. It covers fundamental concepts of rigid body dynamics, control theory, and inverse kinematics, culminating in a simulation of a **UR5e industrial manipulator**.

## 🛠️ Tech Stack
* **Physics Engine:** MuJoCo
* **Language:** Python 3
* **Libraries:** `mujoco`, `numpy`, `gymnasium`, `stable-baselines3`
* **Robots:** Universal Robots UR5e (via MuJoCo Menagerie)

## 📂 Project Structure

| File | Description |
| :--- | :--- |
| `lesson1_basics.py` | Introduction to XML (MJCF) bodies and geoms. |
| `lesson2_pendulum.py` | Simulating a double pendulum dynamics. |
| `lesson3_motor.py` | Actuation and joint control torque. |
| `lesson4_sensors.py` | Reading sensor data (qpos, qvel) and equilibrium states. |
| `lesson5_friction.py` | Contact physics, friction parameters, and stability. |
| `lesson6_ur5.py` | Loading the **UR5e** mesh model from DeepMind's Menagerie. |
| `lesson7_ik.py` | **Inverse Kinematics** implementation using Mocap bodies to make the arm track a target and interact with objects. |

## 🚀 How to Run
1. Clone the repository:
   ```bash
   git clone [https://github.com/RishabhShahIITGN/learning-mujoco.git](https://github.com/RishabhShahIITGN/learning-mujoco.git)
   cd learning-mujoco
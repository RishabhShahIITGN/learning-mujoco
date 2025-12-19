import mujoco
import mujoco.viewer
import time

xml = """
<mujoco>
  <option gravity="0 0 -9.81" />
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>

    <body pos="0 0 2">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size=".05" fromto="0 0 0 0 0 -1" rgba="1 0 0 1"/>
      <body pos="0 0 -1">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="capsule" size=".05" fromto="0 0 0 0 0 -1" rgba="0 1 0 1"/>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="my_motor" joint="shoulder" gear="200"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation Started...")
    
    while viewer.is_running():
        # 1. APPLY CONSTANT FORCE
        # We apply a constant signal of 0.2
        # Torque = 0.2 * 200 (gear) = 40 Nm
        data.ctrl[0] = 0.2
        
        # 2. STEP PHYSICS
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 3. READ SENSORS (Print every 100 steps to avoid spamming)
        # We access data.qpos[0] because 'shoulder' is the 0th joint.
        # Check if the step number is divisible by 50
        # (This just slows down the printing so you can read it)
        if data.time % 0.5 < 0.01: 
             # qpos is in Radians. 
             # 0 = Vertical Down
             # 1.57 = Horizontal (90 degrees)
             print(f"Time: {data.time:.2f}s | Angle: {data.qpos[0]:.2f} rad | Velocity: {data.qvel[0]:.2f} rad/s")

        time.sleep(0.005)
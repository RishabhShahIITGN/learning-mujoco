import mujoco
import mujoco.viewer
import time
import math

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
    <motor name="my_motor" joint="shoulder" gear="500"/>
  </actuator>

</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        # 3. CONTROL LOOP
        # We want the motor to oscillate (swing back and forth)
        # We use a sine wave: sin(time)
        
        current_time = time.time() - start_time
        control_signal = math.sin(current_time * 2) # Speed of oscillation
        
        # Apply the control to the actuator
        data.ctrl[0] = control_signal
        
        # Step the physics
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)
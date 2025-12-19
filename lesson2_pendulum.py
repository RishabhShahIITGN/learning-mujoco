import mujoco
import mujoco.viewer
import time

# MJCF Definition of a Double Pendulum
xml = """
<mujoco>
  <option gravity="0 0 -9.81" />
  
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>

    <body pos="0 0 2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="capsule" size=".05" fromto="0 0 0 0 0 -1" rgba="1 0 0 1"/>

      <body pos="0 0 -1">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="capsule" size=".05" fromto="0 0 0 0 0 -1" rgba="0 1 0 1"/>
      </body>
      
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Let's give it a tiny push so it starts moving vigorously
    data.qvel[0] = 5  # Set velocity of the first joint to 5 rad/s
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)
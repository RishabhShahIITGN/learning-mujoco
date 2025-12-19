import mujoco
import mujoco.viewer
import time

xml = """
<mujoco>
  <option gravity="0 0 -9.81" />
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1" />
    
    <geom type="plane" size="2 2 0.1" rgba=".9 .9 .9 1" friction="1.0 0.005 0.0001"/>
    
    <body pos="0 0 0.1">
      <joint type="free"/>
      <geom type="box" size=".1 .1 .1" rgba="0 0 1 1"/>
    </body>

  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # GIVE IT A PUSH!
    # qvel indices: 0,1,2 are linear velocity (x,y,z). 3,4,5 are rotational.
    # We set X-velocity to 5 m/s
    data.qvel[0] = 5 
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)
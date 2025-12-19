import mujoco
import mujoco.viewer
import time

# 1. Define the Robot/System (MJCF - XML format)
# This XML defines a floor (plane) and a red ball (sphere) above it.
xml = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="sphere" size=".1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# 2. Compile the Model
# The 'model' contains static information (physics constants, geometry).
model = mujoco.MjModel.from_xml_string(xml)

# 3. Create the Data
# The 'data' contains the dynamic state (position, velocity, forces).
data = mujoco.MjData(model)

# 4. Launch the Viewer and Simulation Loop
# This opens a window to visualize the simulation.
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation started. Close the viewer window to stop.")
    
    # Run until the viewer is closed
    while viewer.is_running():
        # Step the physics (advance time by 2ms, the default timestep)
        mujoco.mj_step(model, data)

        # Sync the viewer with the current physics state
        viewer.sync()
        
        # Slow down slightly so we can see it (optional, for human eyes)
        time.sleep(0.005)
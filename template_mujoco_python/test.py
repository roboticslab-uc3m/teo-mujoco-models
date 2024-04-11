import mediapy as media
import mujoco
import mujoco_py
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

xml = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -30">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

# Change to a random color the red box
model.geom('red_box').rgba[:3] = np.random.rand(3)

simulation = 3.8
framerate   = 60

frames = []
mujoco.mj_resetData(model, data)    # Reset state and time
while data.time < simulation:
    mujoco.mj_step(model, data)
    if len(frames) < data.time * framerate:
        renderer.update_scene(data)
        image = renderer.render()
        frames.append(image)

# Convert frames to a video
video = np.stack(frames)

fig = plt.figure()

def update(i):
    plt.imshow(frames[i])

ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=1000/framerate)
plt.show()
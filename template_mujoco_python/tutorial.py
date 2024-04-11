
import mediapy as media
import mujoco
import mujoco_py
import matplotlib.pyplot as plt
import numpy as np

xml ="""
<mujoco>
  <worldbody>

    <light name="top" pos="0 0 1"/>
      
    <body name="head" pos="0 0 0">
        <geom name="head" type="sphere" pos="0 0 0" size=".09"/>
        <joint name="frontalNeck_joint" type="hinge" pos="0 0 0" axis="0 1 0" range="-10 10" stiffness="1" armature="0.0068"/>
        <geom name="frontalNeck_geom" type="capsule" fromto="0 -0.05 -0.17 0 0.05 -0.17" size="0.035"/>
        <joint name="axialNeck_joint" type="hinge" pos="0 0 0" axis="0 0 1" range="-60 60" stiffness="1" armature="0.0068"/>
        <geom name="axialNeck_geom" type="capsule" fromto="0 0 -0.25 0 0 -0.27" size="0.035"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco_py.load_model_from_xml(xml)
sim = mujoco_py.MjSim(model)

viewer = mujoco_py.MjViewer(sim)
viewer.cam.trackbodyid = 1
viewer.cam.distance = 1

head_id = model.geom_name2id('head')
model.geom_rgba[head_id, :3] = [1, 1, 0]

while True:
    sim.step()
    # Get's the ID of the head geom
    #joint_id = model.joint_name2id('frontalNeck_joint')

    # Moves the head geom
    #sim.data.qpos[joint_id] += 0.01
    viewer.render()
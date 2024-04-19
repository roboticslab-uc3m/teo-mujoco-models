# Objectives of Task
1. Install MuJoCo
2. Make a TEO XML model for MuJoCo
# Why are we developing this?
![[imgs/mujoco_logo.png]]
**MuJoCo** is a free and open source physics engine that aims to facilitate research and development in robotics and other areas where fast and accurate simulation is needed.

This simulator has been getting popular during the last years because of it's use in the field of reinforcement learning.

We want to implement MuJoCo because the idea is to create a new TEO model, following the XML format of MuJoCo and make it learn how to walk using reinforcement learning.

# Install MuJoCo
When I started to read about how to install and use MuJoCo I found that there is a package called *mujoco-py* that allows using MuJoCo from Python 3. 
This package is deprecated and only supports MuJoCo 2.1 that is an old version.
Right now there are official and maintained [Python bindings for the MuJoCo physics engine.](https://github.com/google-deepmind/mujoco/blob/main/python/README.md) 
This installation process is for **Ubuntu 22.04 with x86_64 architecture**. Let's get started with the installation:
1. Download the latest [binaries](https://github.com/google-deepmind/mujoco/releases).
2. Extract the binaries:
```bash
cd ~/Downloads
tar -xf mujoco-<version>-linux-x86_64.tar.gz
```
3.  Move the extracted folder to home:
```bah
mv mujoco-<version> ~
```
4. Check the installation:
```bash
cd ~/mujoco-<version>/bin
./simulate ../model/humanoid/humanoid.xml
```
You should see something like this:
![[imgs/mujoco_installation.png]]
# Make TEO XML Model
To make the model of TEO I've follow this diagram:
![[imgs/teo_diagram.png]]
In contrast to the Gazebo SDF format, MuJoCo employs the XML format. Unlike SDF files, XML files in MuJoCo do not utilize the "parent" and "child" tags to delineate the structure of TEO's entire body. Instead, it follows a tree-like structure, where the parent node is the AxialTrunk, which branches into three nodes: FrontalTrunk, AxialRightHip, and AxialLeftHip. The FrontalTrunk, in turn, branches into three nodes: AxialNeck, FrontalRightShoulder, and FrontalLeftShoulder.

# Use TEO XML Model in MuJoCo
To use the TEO model you should do as follows:
```bash
# Only if you have a repos folder
cd ~/repos
git clone https://github.com/roboticslab-uc3m/teo-mujoco-models.git
cd ~/mujoco-<version>/bin
./simulate ~/repos/teo-mujoco-models/models/teo_blocks.xml
```
You should see something like this:
![[imgs/teo_blocks_mujoco.png]]
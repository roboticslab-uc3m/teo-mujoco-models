# teo-mujoco-models

## Install MuJoCo
Download the MuJoCo binaries from: `https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz`

Extract the downloaded file to our new `.mujoco` folder:
```bash
mkdir ~/.mujoco
tar -xf Downloads/mujoco210-linux-x86_64.tar.gz ~/.mujoco
```

Install using `pip`:
```bash
pip3 install -U 'mujoco-py<2.2,>=2.1
```

Add to the following line to `.bashrc`:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/YOUR_USER/.mujoco/mujoco210/bin
```

Try MuJoCo:
```bash
cd /home/YOUR_USER/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml 
```

You should see something like this:
![Screenshot from 2024-04-04 09-29-29](https://github.com/roboticslab-uc3m/teo-mujoco-models/assets/38068010/bf833a6f-d498-4b29-b0aa-5901ce868e5b)

## Here what I am learning

### Running MuJoCo in model visualizer
```bash
cd ~/.mujoco/mujoco210/bin
./simulate
```

### Trying to make the TEO model work in MuJoCo

I added the `teo_model` folder, this folder belongs to `teo-gazebo-models`.
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
file:///home/armega/Pictures/Screenshots/Screenshot%20from%202024-04-04%2009-29-29.png
-28:-47:84:48

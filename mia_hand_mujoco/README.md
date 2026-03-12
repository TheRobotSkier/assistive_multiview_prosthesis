# mia_hand_mujoco

This package allows to simulate Mia Hand in [MuJoCo](https://mujoco.org/) and to 
control the simulated hand through the [ROS2 Control](https://control.ros.org/jazzy/index.html) 
framework.

## Prerequisites

This package requires MuJoCo system to be installed. A simple procedure to do that 
is:

* Navigate to the MuJoCo releases [page](https://github.com/google-deepmind/mujoco/releases).
* Download the archive `mujoco-x.x.x-linux_x86_64.tar.gz`, where `x.x.x` denotes the target version.
* Extract the files in the archive, through `tar -xf mujoco-x.x.x-linux_x86_64.tar.gz`.
* Move the extracted `mujoco-x.x.x` folder to a preferred location. Two common 
  destinations are `/usr/local/` or `/opt/`.

An alternative way to install MuJoCo is to build it from source. The procedure is 
described in the MuJoCo [documentation](https://mujoco.readthedocs.io/en/stable/programming/index.html#building-from-source).

After installing MuJoCo, MuJoCo `bin/` directory can be optionally added to `PATH`, 
by adding `export PATH="$PATH:/PATH_TO_MUJOCO_DIR/mujoco-x.x.x/bin"` to `.bashrc` 
script.

Before compiling the package, the MUJOCO_DIR variable need to be exported and the 
`lib/` directory needs to be added to LD_LIBRARY_PATH:

```
export MUJOCO_DIR="/PATH_TO_MUJOCO_DIR/mujoco-x.x.x/"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/$MUJOCO_DIR/lib"
```

## Launching Mia Hand simulated system interface

To launch Mia Hand simulated system interface, and spawn the related controllers, 
the script `mia_hand_system_interface_launch.py` can be used. So, the bash command 
would be `ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py`.
The script takes the following arguments:

* `xml_model_path`: path to the Mia Hand MuJoCo XML model. Defaults to `mia_hand/scene_right.xml`.
* `laterality`: Mia Hand laterality (for URDF loading). Accepts `right` or `left`.
      Defaults to `right`. **Note:** Mia Hand laterality should be consistent with 
      the one of the XML model chosen.
* `prefix`: optional prefix to be added to Mia Hand URDF element names.
* `robot_ns`: namespace assigned to robot-specific nodes. Defaults to `mia_hand`.

This script spawns, for each joint, a position controller.

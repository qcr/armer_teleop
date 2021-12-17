# Armer (Manipulation Package) Teleoperation
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This is a teleoperation driver based on joy for the Armer driver. For the installation and usage of this package, please see the subsequent sections below:

## Supported Controllers
- Logitech Wireless Gamepad F710
- PS4 Wireless Controller ***[In Progress]***

## Installation
Copy and paste the following code snippet into a terminal to create a new catkin workspace and install the armer_teleop to it. Note that, the armer package is not required as a dependency, as the package can be run from an external camera. If directly used with Armer, it is recommended to be placed in the same catkin_ws for ease of use.

```
mkdir -p ~/armer_ws/src && cd ~/armer_ws/src
git clone https://github.com/qcr/armer_teleop.git
cd .. && rosdep update && rosdep install --from-paths src --ignore-src -r -y
catkin_make
echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc
source ~/armer_ws/devel/setup.bash
echo "Installation Complete!"
```

## Usage
The package can be run independently via its launch file:

***Note: that the arguments specified below are the defaults, so this launch file can be run without those arguments. If they are required to be changed, then each argument should be passed in as shown below.***
```
roslaunch armer_teleop teleop.launch frame_id_EE:=tool0 frame_id_base:=base_link trig_val:=true
```

Note that the ***trig_val*** argument should be kept true (default): if this parameter is not set (i.e., false), the triggers on the controller report a value of 0.0 until they are touched.




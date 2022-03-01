# Armer (Manipulation Package) Teleoperation
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS Test Pipeline](https://github.com/qcr/armer_teleop/actions/workflows/ros_test.yml/badge.svg)](https://github.com/qcr/armer_teleop/actions/workflows/ros_test.yml)

This is a teleoperation driver based on joy for the Armer driver. For the installation and usage of this package, please see the subsequent sections below:

## Supported Controllers
- Logitech Wireless Gamepad F710
- PS4 Wireless Controller

## Installation

### Ubuntu
Before running the below command, ensure you have the [QCR Repositories](https://qcr.github.io/armer/add_qcr_repos.html) setup correctly:
```bash
sudo apt update && sudo apt install ros-noetic-armer-teleop
```
Please note the currently supported (built) ROS distros and architectures:
- ubuntu 20.04 (noetic) amd64 
- ubuntu 20.04 (noetic) arm64
- ubuntu 20.04 (noetic) arm32
- ubuntu 18.04 (melodic) amd64 ***[TODO]***

### From Source
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
roslaunch armer_teleop teleop.launch frame_id_EE:=tool0 frame_id_base:=base_link trig_val:=true controller_config:=logitech
```

Note that the ***trig_val*** argument should be kept true (default): if this parameter is not set (i.e., false), the triggers on the controller report a value of 0.0 until they are touched.

Furthermore, note that the ***controller_config*** argument is defaulted to ***logitech***, and can be changed to ***ps4***, as an example, to run the ps4 configured version. Please see above sections for supported controller types




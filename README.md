# Armer (Manipulation Package) Teleoperation
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS Test Pipeline](https://github.com/qcr/armer_teleop/actions/workflows/ros_test.yml/badge.svg)](https://github.com/qcr/armer_teleop/actions/workflows/ros_test.yml)

This is a teleoperation driver based on joy for the Armer driver. For the installation and usage of this package, please see the subsequent sections below:

## Supported Controllers
- Logitech Wireless Gamepad F710
- PS4 Wireless Controller

## Installation

### Ubuntu (via apt)
Before running the below command, ensure you have the [QCR Repositories](https://qcr.github.io/armer/add_qcr_repos.html) setup correctly:
```bash
sudo apt update && sudo apt install ros-noetic-armer-teleop
```

### Ubuntu (via dpkg)
You can also download the latest debian packages by running the following command (in a suitable directory). Note that you must specify the ***ARCH*** you wish to download (i.e., amd64 or arm64). Alternatively, you can manually download from the latest release in this repository.
  
```bash
curl -s https://api.github.com/repos/qcr/armer_teleop/releases/latest \
| grep "browser_download_url.*<ARCH>.deb" \
| cut -d : -f 2,3 \
| tr -d \" \
| wget -qi -
```
To install, run the following command. Note, please replace ***Downloaded Deb*** with your downloaded version:
```bash
sudo dpkg -i <Downloaded Deb>.deb
```
Please note the currently supported (built) ROS distros and architectures:
- ubuntu 20.04 (noetic) amd64 
- ubuntu 20.04 (noetic) arm64
- ubuntu 18.04 (melodic) amd64
- ubuntu 18.04 (melodic) arm64

### From Source
Copy and paste the following code snippet into a terminal to create a new catkin workspace and install the armer_teleop to it. Note that, the armer package is not required as a dependency, as the package can be run from an external camera. If directly used with Armer, it is recommended to be placed in the same catkin_ws for ease of use.

```bash
mkdir -p ~/armer_ws/src && cd ~/armer_ws/src
git clone https://github.com/qcr/armer_teleop.git
cd .. && rosdep update && rosdep install --from-paths src --ignore-src -r -y
catkin_make
echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc
source ~/armer_ws/devel/setup.bash
echo "Installation Complete!"
```

## Usage
***WARNING: Please Ensure A Safe and Clear Workspace If Using This Package With A Real Robot***

The package can be run independently via its launch file:

***Note: that the arguments specified below are the defaults, so this launch file can be run without those arguments. If they are required to be changed, then each argument should be passed in as shown below.***

```bash
roslaunch armer_teleop teleop.launch frame_id_EE:=tool0 frame_id_base:=base_link trig_val:=true controller_config:=logitech
```

Note that the ***trig_val*** argument should be kept true (default): if this parameter is not set (i.e., false), the triggers on the controller report a value of 0.0 until they are touched.

Furthermore, note that the ***controller_config*** argument is defaulted to ***logitech***, and can be changed to ***ps4***, as an example, to run the ps4 configured version. Please see above sections for supported controller types

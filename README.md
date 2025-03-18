# robotiq_hande_driver
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Package for controlling the [Robotiq Hand-E gripper](https://robotiq.com/products/adaptive-grippers#Hand-E) using the [ROS 2 Control](https://control.ros.org/humble/doc/getting_started/getting_started.html) framework. It uses the [robotiq_hande_description](https://github.com/AGH-CEAI/robotiq_hande_description) package for URDF definitions. Originally developed for integration with Universal Robots e-Series (UR5e) and ROS 2 Humble, it is possible to modify this repository to fit your needs. **PRs are welcome!**

---

## Quick Start

### Connection Setup

#### Without Physical Hardware
* You are ready to go - just remember to pass the `use_fake_hardware:=true` argument.

#### Modbus RTU
* Connect the serial port to your system and locate the TTY device (typically `/dev/ttyX`).
* You will need to configure the serial connection in your `.xacro.urdf` file ([example](https://github.com/AGH-CEAI/robotiq_hande_description/blob/humble/urdf/robotiq_hande_gripper.urdf.xacro) in `robotiq_hande_description`).

#### Modbus TCP
* You can create a local `TCP` <-> `Virtual Serial Port` server with the `socat` command.
  * An example usage (in the form of a Python ROS 2 wrapper) can be found in `ur_robot_driver`'s [scripts/tool_communication.py](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/204e215c8a7371f6357e6a09f7e106364e566931/ur_robot_driver/scripts/tool_communication.py#L64).
  * Example: `socat pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave tcp:192.168.1.2:54321`
* For the UR's `ur_robot_driver`, the default path to the virtual serial port is `/tmp/ttyUR`.


### Package setup
```bash
cd ~/ceai_ws/src
git clone git@github.com:AGH-CEAI/robotiq_hande_driver.git ./src
vcs import src < src/robotiq_hande_driver/robotiq_hande_driver.repos &&
colcon build --symlink-install --packages-select robotiq_hande_driver
source ./install/local_setup.sh
```
> [!NOTE]
> **Configuration is split between two packages:**
> * `robotiq_hande_description`: Hardware connection parameters are set in the URDF file. [Example config file](https://github.com/AGH-CEAI/robotiq_hande_description/blob/humble/urdf/robotiq_hande_gripper.urdf.xacro).
> * `robotiq_hande_driver`: The controller configuration is set by launch parameters for the `controller_node`. [Example config file](robotiq_hande_driver/bringup/config/hande_controller.yaml).


### Launch preview
```bash
ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=true
# In other terminal
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \
"command:
  position: 0.0
  max_effort: 0.0
"
```

### Examples
**An example integration usage can be find** in the [AGH-CEAI/aegis_ros](https://github.com/AGH-CEAI/aegis_ros) repository.

---
## Development notes

This project uses various tools for aiding the quality of the source code. Currently most of them are executed by the `pre-commit`. Please make sure to enable its hooks:

```bash
pre-commit install
```

### Test Serial Connection

There is an additional test tool with hardcoded parameters to test the connection with the Hand-E without any ROS dependencies.
Before build, change the hardcoded parameters in the beginning of the `robotiq_hande_driver/test/communication_test.cpp` file.

To run test:
```bash
cd cd ~/ceai_ws/src/robotiq_hande_driver/build/robotiq_hande_driver/
./communication_test
```

---
## License
This repository is licensed under the Apache 2.0, see LICENSE for details.

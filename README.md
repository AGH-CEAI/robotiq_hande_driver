# robotiq_hande_driver
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Package for controlling the [Robotiq Hand-E gripper](https://robotiq.com/products/adaptive-grippers#Hand-E) from the [ROS 2 Control](https://control.ros.org/humble/doc/getting_started/getting_started.html) framework. It uses the [robotiq_hande_description](https://github.com/macmacal/robotiq_hande_description) package for the URDF definitions.

Work is still in progress.


---

## Quick start

### Setup
```bash
cd ~/ceai_ws/src
git clone git@github.com:AGH-CEAI/robotiq_hande_driver.git ./src
vcs import src < src/robotiq_hande_driver/robotiq_hande_driver.repos &&
colcon build --symlink-install --packages-select robotiq_hande_driver
source ./install/local_setup.sh
```

### Launch preview
```bash
ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=true
```

### Test Serial Connection

Another terminal
```bash
cd cd ~/ceai_ws/src/robotiq_hande_driver/build/robotiq_hande_driver/
./communication_test
./hande_driver
```

---
## Development notes

This project uses various tools for aiding the quality of the source code. Currently most of them are executed by the `pre-commit`. Please make sure to enable its hooks:

```bash
pre-commit install
```

---
## License
This repository is licensed under the Apache 2.0, see LICENSE for details.

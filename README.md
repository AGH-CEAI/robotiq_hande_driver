# robotiq_hande_driver

Package for controlling the [Robotiq Hand-E gripper](https://robotiq.com/products/adaptive-grippers#Hand-E) from the [ROS 2 Control](https://control.ros.org/humble/doc/getting_started/getting_started.html) framework.

Work is still in progress.

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

---

## Quick start

### Setup
```bash
cd ~/ceai_ws/src
git clone git@github.com:AGH-CEAI/robotiq_hande_driver.git ./src
colcon build --symlink-install --packages-select robotiq_hande_driver
source ./install/local_setup.sh
```

### Run
```bash
ros2 run robotiq_hande_driver hande_driver
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

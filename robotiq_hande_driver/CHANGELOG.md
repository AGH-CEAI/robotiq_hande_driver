# Changelog

All notable changes to the `robotiq_hande_driver` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2025-08-25

### Added

* [PR-27](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/27) - Added additional launch arguments and provided better instructions in README.
* [PR-21](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/21) - Added:
  * `SocatManager` for managing the external `socat` process.
  * Added colored logging.
  * Introduced 4 new params for URDFs:
    * `frequency_hz` (int) to control the amount of sleep rate in new thread
    * `create_socat_tty` (bool) to create a virtual serial port,
    * `ip_adress` and `port` to configure the creation of virtual serial port with `socat`,

### Changed

* [PR-26](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/26) - Migration from ROS 2 Humble to ROS 2 Jazzy.
* [PR-27](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/27) - Renamed `ip_adress` to `socat_ip_address` and `port` to `socat_port`.
* [PR-21](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/21) - Refactored the modbus communication to use multithreads:
  * Renamed `application.hpp/cpp` to `hande_gripper.hpp/cpp`.
  * Changed plain arrays `uint8_t bytes_[]` into `std::array<uint8_t, *>`.
  * Moved all logic from the `communication.cpp/hpp` to `protocol_logic.cpp/hpp`.
  * Moved all non-trivial definitions from `*.hpp`s to `*.cpp`s.
  * Moved from C-like functions to C++ ones (e.g. to `chrono` and `thread` instead of `usleep()`).
  * Encapsulated whole `Communication` config into `CommunicationConfig` struct.
  *  renamed `tty` param to `tty_port` for better clarity.

### Removed

* [PR-24](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/24) - Cleanup of unused variables.

### Fixed

* [PR-25](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/25) - Fixed wrong type for the force command interface (from `HW_IF_POSITION` to `HW_IF_EFFORT`).
* [PR-21](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/21) - Fixed:
  * Fixed integration with - UR's RTDE communication protocol ([aegis_ros#38](https://github.com/AGH-CEAI/aegis_ros/issues/38)).
  * Re-enabled the `-Werror` flag #5.
  * Fixed typos with wrong values in  `GRIPPER_OUTPUT_FIRST_REG`  and `GRIPPER_INPUT_FIRST_REG`.
* [PR-16](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/16) - Fixed URDF typos in the `test_load_hw_interface.cpp` file.

## [0.1.0] - 2025-03-13

### Added

* [PR-11](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/11) - Added automatic reconnection to Modbus during configuration.
* [PR-7](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/7) - Integration of ModbusRTU communication with ros2_control Hardware Interface
* [PR-2](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/2) - ModbusRTU communication
* [PR-2](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/2) - Protocol abstraction
* [PR-1](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/1) - Prepared ros2_control Hardware Interface.

### Fixed

* [PR-11](https://github.com/AGH-CEAI/robotiq_hande_driver/pull/11) - Fixed an issue where the Modbus connection check always failed.

cmake_minimum_required(VERSION 3.16)
project(robotiq_hande_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  # If you got problems with ROS Humble's bundled gmock version: a) change the compiler to clang >=
  # 14.0.0 b) add flag `-Wno-deprecated-copy`
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

include(FindPkgConfig)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

set(DEPENDS modbus)

set(HW_IF_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    rcpputils)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME} SHARED
  hardware/src/hande_hardware_interface.cpp
  hardware/src/hande_gripper.cpp
  hardware/src/communication.cpp
  hardware/src/protocol_logic.cpp
  hardware/src/socat_manager.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDS})

# TODO(issue#8) Build the communication_test only with BUILD_TESTING flag
add_executable(communication_test test/communication_test.cpp)
target_link_libraries(communication_test ${catkin_LIBRARIES} ${DEPENDS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  robotiq_hande_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
                              $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${HW_IF_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface ros2_control_plugin.xml)

install(DIRECTORY hardware/include/ DESTINATION include/)
install(DIRECTORY bringup DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  # find_package(ament_cmake_pytest REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gmock(test_load_hw_interface test/test_load_hw_interface.cpp)
  target_link_libraries(test_load_hw_interface ${PROJECT_NAME})
  ament_target_dependencies(test_load_hw_interface ros2_control_test_assets)

  # TODO(issue#8) Build the communication_test only with BUILD_TESTING flag
  # ament_add_gmock(application_test test/application_test.cpp )
  # target_link_libraries(application_test ${PROJECT_NAME})
endif()

# EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()

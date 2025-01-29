cmake_minimum_required(VERSION 3.16)
project(robotiq_hande_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(HW_IF_INCLUDE_DEPENDS
  fmt
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
    ${PROJECT_NAME}
    SHARED
    hardware/src/hande_hardware_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  robotiq_hande_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface ros2_control_plugin.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  # find_package(ament_cmake_pytest REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gmock(test_load_hw_interface
    test/test_load_hw_interface.cpp
  )
  target_link_libraries(test_load_hw_interface ${PROJECT_NAME})

  # TODO: investigate
  # ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
  # ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
  # ament_add_pytest_test(run_example_7_launch test/test_r6bot_controller_launch.py)
endif()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()

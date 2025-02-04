cmake_minimum_required(VERSION 3.8)
project(robotiq_hande_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(
  "include"
  "src"
)

add_executable(hande_driver src/hande_driver.cpp src/application.cpp src/protocol_logic.cpp src/communication.cpp)
target_link_libraries(hande_driver ${catkin_LIBRARIES})
target_link_libraries(hande_driver /lib/x86_64-linux-gnu/libmodbus.so)

add_executable(communication_test src/communication_test.c)
target_link_libraries(communication_test ${catkin_LIBRARIES})
target_link_libraries(communication_test /lib/x86_64-linux-gnu/libmodbus.so)

target_include_directories(hande_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hande_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS hande_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robotiq_hande_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)

add_library(
  include/application.hpp
  include/protocol_logic.hpp
  include/communication.hpp
  src/application.cpp
  src/protocol_logic.cpp
  src/communication.cpp
)

add_executable(hande_driver src/hande_driver.cpp)
target_include_directories(hande_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hande_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS hande_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()

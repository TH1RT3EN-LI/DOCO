cmake_minimum_required(VERSION 3.8)
project(ugv_controller_emulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(ugv_controller_emulator_core
  src/controller_emulator_core.cpp
)
target_compile_features(ugv_controller_emulator_core PUBLIC cxx_std_17)
target_include_directories(ugv_controller_emulator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(ugv_controller_emulator_node
  src/controller_emulator_node.cpp
)
target_compile_features(ugv_controller_emulator_node PUBLIC cxx_std_17)
target_include_directories(ugv_controller_emulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ugv_controller_emulator_node
  rclcpp
  geometry_msgs
  nav_msgs
)
target_link_libraries(ugv_controller_emulator_node
  ugv_controller_emulator_core
  util
)

install(TARGETS
  ugv_controller_emulator_core
  ugv_controller_emulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(uav_gz_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(uav_bridge REQUIRED)

add_executable(mono_camera_bridge_node
  src/mono_camera_bridge_node.cpp
)
target_compile_features(mono_camera_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(mono_camera_bridge_node
  rclcpp
  sensor_msgs
  uav_bridge
)
target_link_libraries(mono_camera_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(stereo_camera_bridge_node
  src/stereo_camera_bridge_node.cpp
)
target_compile_features(stereo_camera_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(stereo_camera_bridge_node
  rclcpp
  sensor_msgs
  uav_bridge
)
target_link_libraries(stereo_camera_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(tf_bridge_node
  src/tf_bridge_node.cpp
)
target_compile_features(tf_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(tf_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  uav_bridge
)
target_link_libraries(tf_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(rangefinder_bridge_node
  src/rangefinder_bridge_node.cpp
)
target_compile_features(rangefinder_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(rangefinder_bridge_node
  rclcpp
  px4_msgs
  uav_bridge
)
target_link_libraries(rangefinder_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(optical_flow_bridge_node
  src/optical_flow_bridge_node.cpp
)
target_compile_features(optical_flow_bridge_node PUBLIC cxx_std_17)
target_include_directories(optical_flow_bridge_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(optical_flow_bridge_node
  rclcpp
  px4_msgs
  uav_bridge
)
target_link_libraries(optical_flow_bridge_node
  gz-transport13::core
  gz-msgs10::core
  ${OpenCV_LIBS}
)

install(TARGETS
  mono_camera_bridge_node
  stereo_camera_bridge_node
  tf_bridge_node
  rangefinder_bridge_node
  optical_flow_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

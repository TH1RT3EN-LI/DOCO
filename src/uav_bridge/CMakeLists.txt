cmake_minimum_required(VERSION 3.8)
project(uav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(mono_camera_bridge_node
  src/mono_camera_bridge_node.cpp
)
target_compile_features(mono_camera_bridge_node PUBLIC cxx_std_17)
target_include_directories(mono_camera_bridge_node PRIVATE include)
ament_target_dependencies(mono_camera_bridge_node
  rclcpp
  sensor_msgs
)
target_link_libraries(mono_camera_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(stereo_camera_bridge_node
  src/stereo_camera_bridge_node.cpp
)
target_compile_features(stereo_camera_bridge_node PUBLIC cxx_std_17)
target_include_directories(stereo_camera_bridge_node PRIVATE include)
ament_target_dependencies(stereo_camera_bridge_node
  rclcpp
  sensor_msgs
)
target_link_libraries(stereo_camera_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(tf_bridge_node
  src/tf_bridge_node.cpp
)
target_compile_features(tf_bridge_node PUBLIC cxx_std_17)
target_include_directories(tf_bridge_node PRIVATE include)
ament_target_dependencies(tf_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(tf_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(offboard_bridge_node
  src/offboard_bridge_node.cpp
  src/frame_aligner.cpp
  src/flight_mode_manager.cpp
  src/px4_commander.cpp
  src/math_utils.cpp
)
target_compile_features(offboard_bridge_node PUBLIC cxx_std_17)
target_include_directories(offboard_bridge_node PRIVATE include)
ament_target_dependencies(offboard_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  quadrotor_msgs
  px4_msgs
)
target_link_libraries(offboard_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(rangefinder_bridge_node
  src/rangefinder_bridge_node.cpp
)
target_compile_features(rangefinder_bridge_node PUBLIC cxx_std_17)
target_include_directories(rangefinder_bridge_node PRIVATE include)
ament_target_dependencies(rangefinder_bridge_node
  rclcpp
  px4_msgs
)
target_link_libraries(rangefinder_bridge_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(optical_flow_bridge_node
  src/optical_flow_bridge_node.cpp
)
target_compile_features(optical_flow_bridge_node PUBLIC cxx_std_17)
target_include_directories(optical_flow_bridge_node PRIVATE include)
target_include_directories(optical_flow_bridge_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(optical_flow_bridge_node
  rclcpp
  px4_msgs
)
target_link_libraries(optical_flow_bridge_node
  gz-transport13::core
  gz-msgs10::core
  ${OpenCV_LIBS}
)

add_executable(px4_planar_state_reader_node
  src/px4_planar_state_reader_node.cpp
)
target_compile_features(px4_planar_state_reader_node PUBLIC cxx_std_17)
ament_target_dependencies(px4_planar_state_reader_node
  rclcpp
  nav_msgs
  px4_msgs
)

add_executable(fmu_topic_namespace_bridge_node
  src/fmu_topic_namespace_bridge_node.cpp
)
target_compile_features(fmu_topic_namespace_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(fmu_topic_namespace_bridge_node
  rclcpp
  px4_msgs
)

install(TARGETS
  mono_camera_bridge_node
  stereo_camera_bridge_node
  tf_bridge_node
  offboard_bridge_node
  rangefinder_bridge_node
  optical_flow_bridge_node
  px4_planar_state_reader_node
  fmu_topic_namespace_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

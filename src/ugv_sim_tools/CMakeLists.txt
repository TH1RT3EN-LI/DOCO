cmake_minimum_required(VERSION 3.8)
project(ugv_sim_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/clock_guard_logic.cpp
  src/initial_pose_logic.cpp
  src/odom_to_tf_logic.cpp
  src/point_cloud_utils.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  sensor_msgs
)

add_executable(joint_state_stamp_fix_node
  src/joint_state_stamp_fix_node.cpp
)
target_link_libraries(joint_state_stamp_fix_node ${PROJECT_NAME}_core)
ament_target_dependencies(joint_state_stamp_fix_node
  rclcpp
  sensor_msgs
)

add_executable(scan_frame_rewriter
  src/scan_frame_rewriter_node.cpp
)
target_link_libraries(scan_frame_rewriter ${PROJECT_NAME}_core)
ament_target_dependencies(scan_frame_rewriter
  rclcpp
  sensor_msgs
)

add_executable(odom_to_tf_node
  src/odom_to_tf_node.cpp
)
target_link_libraries(odom_to_tf_node ${PROJECT_NAME}_core)
ament_target_dependencies(odom_to_tf_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

add_executable(initial_pose_publisher
  src/initial_pose_publisher_node.cpp
)
target_link_libraries(initial_pose_publisher ${PROJECT_NAME}_core)
ament_target_dependencies(initial_pose_publisher
  geometry_msgs
  rclcpp
)

add_executable(clock_guard_node
  src/clock_guard_node.cpp
)
target_link_libraries(clock_guard_node ${PROJECT_NAME}_core)
ament_target_dependencies(clock_guard_node
  rclcpp
  rosgraph_msgs
)

add_executable(orbbec_topic_compat_node
  src/orbbec_topic_compat_node.cpp
)
target_link_libraries(orbbec_topic_compat_node ${PROJECT_NAME}_core)
ament_target_dependencies(orbbec_topic_compat_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    joint_state_stamp_fix_node
    scan_frame_rewriter
    odom_to_tf_node
    initial_pose_publisher
    clock_guard_node
    orbbec_topic_compat_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_initial_pose_logic
    test/test_initial_pose_logic.cpp
  )
  if(TARGET test_initial_pose_logic)
    target_link_libraries(test_initial_pose_logic ${PROJECT_NAME}_core)
    ament_target_dependencies(test_initial_pose_logic geometry_msgs)
  endif()

  ament_add_gtest(test_odom_to_tf_logic
    test/test_odom_to_tf_logic.cpp
  )
  if(TARGET test_odom_to_tf_logic)
    target_link_libraries(test_odom_to_tf_logic ${PROJECT_NAME}_core)
  endif()
endif()

ament_package()

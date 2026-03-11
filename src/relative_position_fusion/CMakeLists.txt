cmake_minimum_required(VERSION 3.8)
project(relative_position_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/diagnostics_publisher.cpp
  src/relative_position_filter.cpp
  src/relocalization_monitor.cpp
  src/time_alignment_buffer.cpp
  src/uav_state_adapter.cpp
  src/ugv_state_adapter.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_core
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(relative_position_fusion_node
  src/relative_position_fusion_node.cpp
)
target_include_directories(relative_position_fusion_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(relative_position_fusion_node ${PROJECT_NAME}_core)
ament_target_dependencies(relative_position_fusion_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    relative_position_fusion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_relative_position_filter
    test/test_relative_position_filter.cpp
  )
  if(TARGET test_relative_position_filter)
    target_link_libraries(test_relative_position_filter ${PROJECT_NAME}_core)
    target_include_directories(test_relative_position_filter PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()

  ament_add_gtest(test_relocalization_monitor
    test/test_relocalization_monitor.cpp
  )
  if(TARGET test_relocalization_monitor)
    target_link_libraries(test_relocalization_monitor ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_time_alignment_buffer
    test/test_time_alignment_buffer.cpp
  )
  if(TARGET test_time_alignment_buffer)
    target_link_libraries(test_time_alignment_buffer ${PROJECT_NAME}_core)
    target_include_directories(test_time_alignment_buffer PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()

  ament_add_gtest(test_uav_state_adapter
    test/test_uav_state_adapter.cpp
  )
  if(TARGET test_uav_state_adapter)
    target_link_libraries(test_uav_state_adapter ${PROJECT_NAME}_core)
    target_include_directories(test_uav_state_adapter PRIVATE ${EIGEN3_INCLUDE_DIRS})
    ament_target_dependencies(test_uav_state_adapter tf2 tf2_ros tf2_geometry_msgs)
  endif()

  ament_add_gtest(test_ugv_state_adapter
    test/test_ugv_state_adapter.cpp
  )
  if(TARGET test_ugv_state_adapter)
    target_link_libraries(test_ugv_state_adapter ${PROJECT_NAME}_core)
    target_include_directories(test_ugv_state_adapter PRIVATE ${EIGEN3_INCLUDE_DIRS})
    ament_target_dependencies(test_ugv_state_adapter tf2 tf2_ros tf2_geometry_msgs)
  endif()
endif()

ament_package()

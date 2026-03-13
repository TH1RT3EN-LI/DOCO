cmake_minimum_required(VERSION 3.8)
project(uav_mode_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(relative_position_fusion REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uav_visual_landing REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SupervisorState.msg"
  "srv/CommandSupervisor.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_core
  src/supervisor_logic.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

add_executable(uav_mode_supervisor_node
  src/uav_mode_supervisor_node.cpp
)
target_compile_features(uav_mode_supervisor_node PUBLIC cxx_std_17)
target_include_directories(uav_mode_supervisor_node PRIVATE
  include
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
)
target_link_libraries(uav_mode_supervisor_node
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
  relative_position_fusion::relative_position_fusion__rosidl_typesupport_cpp
  uav_visual_landing::uav_visual_landing__rosidl_typesupport_cpp
)
add_dependencies(uav_mode_supervisor_node
  ${PROJECT_NAME}
)
ament_target_dependencies(uav_mode_supervisor_node
  diagnostic_msgs
  rclcpp
  relative_position_fusion
  std_msgs
  std_srvs
  uav_visual_landing
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    uav_mode_supervisor_node
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

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_supervisor_logic
    test/test_supervisor_logic.cpp
  )
  if(TARGET test_supervisor_logic)
    target_link_libraries(test_supervisor_logic ${PROJECT_NAME}_core)
    target_include_directories(test_supervisor_logic PRIVATE include)
    target_compile_features(test_supervisor_logic PUBLIC cxx_std_17)
  endif()
endif()

ament_package()

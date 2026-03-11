cmake_minimum_required(VERSION 3.8)
project(uav_visual_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingError.msg"
  "msg/LandingStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(aruco_detector_node
  src/aruco_detector_node.cpp
)
target_compile_features(aruco_detector_node PUBLIC cxx_std_17)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  px4_msgs
  std_msgs
  cv_bridge
  image_transport
)
target_include_directories(aruco_detector_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
)
add_dependencies(aruco_detector_node
  ${PROJECT_NAME}
)
target_link_libraries(aruco_detector_node
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}"
)

add_executable(visual_landing_node
  src/visual_landing_node.cpp
)
target_compile_features(visual_landing_node PUBLIC cxx_std_17)
ament_target_dependencies(visual_landing_node
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  std_msgs
  std_srvs
)

# This target consumes both this package's generated messages and px4_msgs
# interfaces, so wire the generator/ts targets explicitly to avoid missing
# include propagation during incremental workspace builds.
target_include_directories(visual_landing_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
)
add_dependencies(visual_landing_node
  ${PROJECT_NAME}
)
target_link_libraries(visual_landing_node
  "${cpp_typesupport_target}"
  px4_msgs::px4_msgs__rosidl_typesupport_cpp
)

install(TARGETS
  aruco_detector_node
  visual_landing_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

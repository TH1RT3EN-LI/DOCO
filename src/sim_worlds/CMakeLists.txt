cmake_minimum_required(VERSION 3.8)
project(sim_worlds)

find_package(ament_cmake REQUIRED)

# Install world files
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install model files. UAV models live in uav_description to avoid duplicate
# model names across resource paths.
install(DIRECTORY
  models/baylands
  models/coast_water
  models/apriltag
  models/ground_texture
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install bridge / runtime config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

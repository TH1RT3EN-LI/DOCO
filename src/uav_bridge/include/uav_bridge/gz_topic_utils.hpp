#pragma once

#include <string>

namespace uav_bridge::gz_topics {

inline std::string WorldPrefix(const std::string &gz_world_name) {
  return "/world/" + gz_world_name;
}

inline std::string PoseInfo(const std::string &gz_world_name) {
  return WorldPrefix(gz_world_name) + "/pose/info";
}

inline std::string Clock(const std::string &gz_world_name) {
  return WorldPrefix(gz_world_name) + "/clock";
}

inline std::string SensorPrefix(const std::string &gz_world_name,
                                const std::string &model_name,
                                const std::string &link_name,
                                const std::string &sensor_name) {
  return WorldPrefix(gz_world_name) + "/model/" + model_name + "/link/" +
         link_name + "/sensor/" + sensor_name;
}

inline std::string Image(const std::string &gz_world_name,
                         const std::string &model_name,
                         const std::string &link_name,
                         const std::string &sensor_name) {
  return SensorPrefix(gz_world_name, model_name, link_name, sensor_name) +
         "/image";
}

inline std::string DepthImage(const std::string &gz_world_name,
                              const std::string &model_name,
                              const std::string &link_name,
                              const std::string &sensor_name) {
  return SensorPrefix(gz_world_name, model_name, link_name, sensor_name) +
         "/depth_image";
}

inline std::string DepthPoints(const std::string &gz_world_name,
                               const std::string &model_name,
                               const std::string &link_name,
                               const std::string &sensor_name) {
  return DepthImage(gz_world_name, model_name, link_name, sensor_name) +
         "/points";
}

inline std::string Scan(const std::string &gz_world_name,
                        const std::string &model_name,
                        const std::string &link_name,
                        const std::string &sensor_name) {
  return SensorPrefix(gz_world_name, model_name, link_name, sensor_name) +
         "/scan";
}

inline std::string Imu(const std::string &gz_world_name,
                       const std::string &model_name,
                       const std::string &link_name,
                       const std::string &sensor_name) {
  return SensorPrefix(gz_world_name, model_name, link_name, sensor_name) +
         "/imu";
}

} // namespace uav_bridge::gz_topics

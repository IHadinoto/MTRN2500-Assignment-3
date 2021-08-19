// Copyright 2019 Zhihao Zhang License MIT

#ifndef LANDSCAPE_HPP_
#define LANDSCAPE_HPP_

#include "config_parser.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"  // http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html
#include "interfaces.hpp"
#include "rclcpp/rclcpp.hpp"        // http://docs.ros2.org/dashing/api/rclcpp/
#include "sensor_msgs/msg/joy.hpp"  // http://wiki.ros.org/joy
#include "shape_base_multi.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes {
// ReSharper disable once CppClassCanBeFinal
class Landscape : public ShapeBaseMulti {
 public:
  explicit Landscape(
      int id,
      std::shared_ptr<visualization_msgs::msg::MarkerArray> landscape_list_ptr);
};
}  // namespace shapes
#endif  // UAV_HPP_

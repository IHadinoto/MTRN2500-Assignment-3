// Copyright 2019 Zhihao Zhang License MIT

#ifndef UAV_HPP_
#define UAV_HPP_

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
class Uav : public ShapeBaseMulti, public rclcpp::Node {
 public:
  explicit Uav(int id, std::string const& zid,
               assignment3::JoystickConfig config,
               std::chrono::milliseconds refresh_period,
               std::shared_ptr<visualization_msgs::msg::MarkerArray> uav_list);

 private:
  assignment3::JoystickConfig const config_;
  auto joy_callback(sensor_msgs::msg::Joy::UniquePtr joy_message) -> void;
  auto pose_callback() -> void;

  geometry_msgs::msg::TwistStamped::UniquePtr velocity_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_input_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::reference_wrapper<visualization_msgs::msg::Marker>> rotors;
};
}  // namespace shapes
#endif  // UAV_HPP_

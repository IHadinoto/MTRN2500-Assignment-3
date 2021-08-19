// Copyright 2019 Zhihao Zhang License MIT

#ifndef CUBE_LIST_MA_HPP_
#define CUBE_LIST_MA_HPP_

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
class CubeListMA : public ShapeBaseMulti, public rclcpp::Node {
 public:
  explicit CubeListMA(
      std::string const& zid, assignment3::JoystickConfig config,
      std::chrono::milliseconds refresh_period,
      std::shared_ptr<visualization_msgs::msg::MarkerArray> CubeListMA_list);
  auto add_cube(
      int id,
      std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> cube_ptr)
      -> void;
  auto get_block_dropped() -> int;

 protected:
  int gravity = 0;
  int gravity_button_flag = 0;
  int block_dropped = 0;
  auto clear_blocks() -> void;

 private:
  assignment3::JoystickConfig const config_;
  auto joy_callback(sensor_msgs::msg::Joy::UniquePtr joy_message) -> void;
  auto gravity_callback() -> void;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace shapes
#endif  // CubeListMA_HPP_

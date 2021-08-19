// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "cube_list_markerarray.hpp"

#include "rclcpp/rclcpp.hpp"        // http://docs.ros2.org/dashing/api/rclcpp/
#include "sensor_msgs/msg/joy.hpp"  // http://wiki.ros.org/joy
#include "student_helper.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cube.hpp"
#include "prism.hpp"
#include "sphere.hpp"

#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes {
/**
 * \brief Example of how sphere class may be implemented, the design may not be
 *the most suitable. Trivial functionalities are not implemented.
 **/

CubeListMA::CubeListMA(
    std::string const& zid, assignment3::JoystickConfig config,
    std::chrono::milliseconds refresh_period,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> cube_list_ptr)

    : ShapeBaseMulti(cube_list_ptr),
      rclcpp::Node{helper::cube_list_node_name(zid)},
      config_{config.x_axis,
              config.y_axis,
              config.z_plus_axis,
              config.z_minus_axis,
              config.steering_axis,
              config.drop_block_button,
              config.clear_blocks_button,
              config.gravity_button,
              config.trigger_deadzone,
              config.joystick_deadzone,
              config.floor_bound,
              config.ceiling_bound,
              config.side_bound} {
  // Subscribe to Joy
  auto joy_callback_wrapper =
      std::bind(&CubeListMA::joy_callback, this, std::placeholders::_1);
  this->joystick_input_ = create_subscription<sensor_msgs::msg::Joy>(
      std::string("/" + zid + "/joy"), 10, joy_callback_wrapper);

  // Set up wall timer
  auto gravity_callback_wrapper =
      std::bind(&CubeListMA::gravity_callback, this);
  this->timer_ = create_wall_timer(std::chrono::milliseconds{refresh_period},
                                   gravity_callback_wrapper);
}

auto CubeListMA::joy_callback(sensor_msgs::msg::Joy::UniquePtr joy_message)
    -> void {
  gravity_button_flag += joy_message->buttons[config_.gravity_button];
  gravity_button_flag *= joy_message->buttons[config_.gravity_button];
  if (gravity_button_flag >= 2) gravity_button_flag = 2;

  // Toggle gravity on and off
  if (gravity_button_flag == 1) gravity = !gravity;

  // Clear Cubes
  if (joy_message->buttons[config_.clear_blocks_button]) clear_blocks();
  block_dropped = joy_message->buttons[config_.drop_block_button];
}

auto CubeListMA::gravity_callback() -> void {
  if (!gravity) return;
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    if (shapes_list[i].pose.position.z > 0.5)
      shapes_list[i].pose.position.z -= 0.1;
    if (shapes_list[i].pose.position.z < 0.5)
      shapes_list[i].pose.position.z = 0.5;
  }
}

auto CubeListMA::add_cube(
    int id,
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> cube_ptr)
    -> void {
  // Drop blocks on a timer
  auto& shapes_marker_array = shapes_list_list_ptr_->markers;
  auto cubes_list = *cube_ptr;
  cubes_list[0].id = id;
  cubes_list[0].ns = "CUBE_LIST";
  shapes_marker_array.push_back(cubes_list[0]);
}

auto CubeListMA::clear_blocks() -> void {
  auto& shapes_marker_array = shapes_list_list_ptr_->markers;
  shapes_marker_array.clear();
}

auto CubeListMA::get_block_dropped() -> int { return block_dropped; }

}  // namespace shapes

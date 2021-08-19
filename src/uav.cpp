// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "uav.hpp"

#include "rclcpp/rclcpp.hpp"        // http://docs.ros2.org/dashing/api/rclcpp/
#include "sensor_msgs/msg/joy.hpp"  // http://wiki.ros.org/joy
#include "student_helper.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "shape_generator.hpp"

#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes {

Uav::Uav(int id, std::string const& zid, assignment3::JoystickConfig config,
         std::chrono::milliseconds refresh_period,
         std::shared_ptr<visualization_msgs::msg::MarkerArray> uav_list_ptr)

    : ShapeBaseMulti(uav_list_ptr),
      rclcpp::Node{helper::uav_node_name(zid)},
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
  // Get a ref to the vector of marker for ease of use
  auto& shapes_marker_array = shapes_list_list_ptr_->markers;

  //===========================================================================
  //                             SHAPES CREATION
  //===========================================================================

  // Define initial relative position of UAV.
  int position_x = 0;
  int position_y = 0;
  int position_z = 100;

  // Define LED on UAV.
  auto led = shapes::sphere_gen(id++, 1);
  auto& shapes_list = *(led->get_display_markers());
  auto& shape = shapes_list[0];
  shape.ns = "UAV";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 15);
  shape.color = helper::rgb_to_object(1, 1, 0, 1);
  shape.scale = helper::points_to_vector3(0.4, 0.4, 0.4);
  shapes_marker_array.push_back(shape);

  // Define rotor 1 on UAV.
  auto rotor_1 = shapes::cube_gen(id++, 1);
  shapes_list = *(rotor_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 17.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 14, 0.5);
  shapes_marker_array.push_back(shape);

  // Define rotor 2 on UAV.
  auto rotor_2 = shapes::cube_gen(id++, 1);
  shapes_list = *(rotor_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 18);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(14, 0.5, 0.5);
  shapes_marker_array.push_back(shape);

  // Define sleigh leg 1 on UAV.
  auto sleigh_leg_1 = shapes::cube_gen(id++, 1);
  shapes_list = *(sleigh_leg_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_LEGS";
  shape.pose.position =
      helper::points_to_object(position_x + 1.5, position_y - 3, position_z);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(14, 0.5, 0.5);
  shapes_marker_array.push_back(shape);

  // Define sleigh leg 2 on UAV.
  auto sleigh_leg_2 = shapes::cube_gen(id++, 1);
  shapes_list = *(sleigh_leg_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_LEGS";
  shape.pose.position =
      helper::points_to_object(position_x + 1.5, position_y + 3, position_z);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(14, 0.5, 0.5);
  shapes_marker_array.push_back(shape);

  // Define sleigh join 1 on UAV.
  auto sleigh_join_1 = shapes::cylinder_gen(id++, 0.5, 1);
  ;
  shapes_list = *(sleigh_join_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_JOINS";
  shape.pose.position =
      helper::points_to_object(position_x, position_y + 2.9, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 0.5, 1);
  shapes_marker_array.push_back(shape);

  // Define sleigh join 2 on UAV.
  auto sleigh_join_2 = shapes::cylinder_gen(id++, 0.5, 1);
  ;
  shapes_list = *(sleigh_join_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_JOINS";
  shape.pose.position =
      helper::points_to_object(position_x, position_y - 2.9, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 0.5, 1);
  shapes_marker_array.push_back(shape);

  // Define sleigh join 3 on UAV.
  auto sleigh_join_3 = shapes::cylinder_gen(id++, 0.5, 1);
  ;
  shapes_list = *(sleigh_join_3->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_JOINS";
  shape.pose.position = helper::points_to_object(
      position_x - 3, position_y + 2.9, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 0.5, 1);
  shapes_marker_array.push_back(shape);

  // Define sleigh join 4 on UAV.
  auto sleigh_join_4 = shapes::cylinder_gen(id++, 0.5, 1);
  ;
  shapes_list = *(sleigh_join_4->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_JOINS";
  shape.pose.position = helper::points_to_object(
      position_x - 3, position_y - 2.9, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 0.5, 1);
  shapes_marker_array.push_back(shape);

  // Define sleigh join 5 on UAV.
  auto sleigh_join_5 = shapes::cylinder_gen(id++, 0.5, 1);
  ;
  shapes_list = *(sleigh_join_5->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_JOINS";
  shape.pose.position = helper::points_to_object(
      position_x + 3, position_y + 2.9, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 0.5, 1);
  shapes_marker_array.push_back(shape);

  // Define sleigh join 6 on UAV.
  auto sleigh_join_6 = shapes::cylinder_gen(id++, 0.5, 1);
  ;
  shapes_list = *(sleigh_join_6->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_JOINS";
  shape.pose.position = helper::points_to_object(
      position_x + 3, position_y - 2.9, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shape.scale = helper::points_to_vector3(0.5, 0.5, 1);
  shapes_marker_array.push_back(shape);

  // Define Sleigh Base on UAV.
  auto sleigh_base = shapes::flat_plane_gen(id++, 8, 6);
  ;
  shapes_list = *(sleigh_base->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_BASE";
  shape.pose.position =
      helper::points_to_object(position_x + 1, position_y, position_z + 1.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shapes_marker_array.push_back(shape);

  // Define Sleigh Walls on UAV.
  auto sleigh_wall_1 = shapes::rectangular_prism_gen(id++, 8.5, 0.5, 3);
  ;
  shapes_list = *(sleigh_wall_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_WALLS";
  shape.pose.position =
      helper::points_to_object(position_x, position_y + 3, position_z + 1.6);
  shape.color = helper::rgb_to_object(1, 0, 0, 1);
  shapes_marker_array.push_back(shape);

  auto sleigh_wall_2 = shapes::rectangular_prism_gen(id++, 8.5, 0.5, 3);
  ;
  shapes_list = *(sleigh_wall_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_WALLS";
  shape.pose.position =
      helper::points_to_object(position_x, position_y - 3, position_z + 1.6);
  shape.color = helper::rgb_to_object(1, 0, 0, 1);
  shapes_marker_array.push_back(shape);

  auto sleigh_wall_3 = shapes::parallelepiped_gen(id++, 0.7, 6, 3);
  ;
  shapes_list = *(sleigh_wall_3->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_WALLS";
  shape.pose.position =
      helper::points_to_object(position_x - 3, position_y, position_z + 1.6);
  shape.color = helper::rgb_to_object(1, 0, 0, 1);
  shapes_marker_array.push_back(shape);

  auto sleigh_wall_4 = shapes::parallelepiped_gen(id++, 0.7, 6, 3);
  ;
  shapes_list = *(sleigh_wall_4->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_WALLS";
  shape.pose.position =
      helper::points_to_object(position_x + 2, position_y, position_z + 1.6);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shapes_marker_array.push_back(shape);

  // Define Sleigh Wall Cover on UAV.
  auto sleigh_wall_cover_1 = shapes::rectangular_prism_gen(id++, 8.5, 0.7, 0.5);
  shapes_list = *(sleigh_wall_cover_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_FRILL";
  shape.pose.position =
      helper::points_to_object(position_x, position_y + 3, position_z + 4.6);
  shape.color = helper::rgb_to_object(1, 1, 0, 1);
  shapes_marker_array.push_back(shape);

  auto sleigh_wall_cover_2 = shapes::rectangular_prism_gen(id++, 8.5, 0.7, 0.5);
  shapes_list = *(sleigh_wall_cover_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_FRILL";
  shape.pose.position =
      helper::points_to_object(position_x, position_y - 3, position_z + 4.6);
  shape.color = helper::rgb_to_object(1, 1, 0, 1);
  shapes_marker_array.push_back(shape);

  auto sleigh_wall_cover_3 = shapes::rectangular_prism_gen(id++, 0.7, 8, 0.7);
  shapes_list = *(sleigh_wall_cover_3->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_FRILL";
  shape.pose.position =
      helper::points_to_object(position_x - 3, position_y, position_z + 4.6);
  shape.color = helper::rgb_to_object(1, 1, 0, 1);
  shapes_marker_array.push_back(shape);

  // Define Sleigh Guards on UAV.
  auto sleigh_guard_1 = shapes::cube_gen(id++, 0.5);
  ;
  shapes_list = *(sleigh_guard_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_GUARD";
  shape.pose.position = helper::points_to_object(
      position_x + 8.5, position_y + 3, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shapes_marker_array.push_back(shape);

  auto sleigh_guard_2 = shapes::cube_gen(id++, 0.5);
  ;
  shapes_list = *(sleigh_guard_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_GUARD";
  shape.pose.position = helper::points_to_object(
      position_x + 8.5, position_y - 3, position_z + 0.5);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shapes_marker_array.push_back(shape);

  // Santa Cushion Setup.
  auto santa_cushion = shapes::octagonal_prism_gen(id++, 1, 6.2);
  shapes_list = *(santa_cushion->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_CUSHION";
  shape.pose.position = helper::points_to_object(position_x + 4, position_y + 3,
                                                 position_z + 1.5);
  shape.pose.orientation = helper::point_to_quaternion(0, 0, helper::pi / 2);
  shape.color = helper::rgb_to_object(1, 0, 0, 1);
  shapes_marker_array.push_back(shape);

  auto santa_cushion_2 = shapes::triangular_prism_gen(id++, 1, 6);
  shapes_list = *(santa_cushion_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_CUSHION";
  shape.pose.position = helper::points_to_object(
      position_x + 3.2, position_y + 3, position_z + 1.9);
  shape.pose.orientation =
      helper::point_to_quaternion(0, helper::pi / 9, helper::pi / 2);
  shape.color = helper::rgb_to_object(1, 0, 0, 1);
  shapes_marker_array.push_back(shape);

  // Presents for UAV.
  auto present_1 = shapes::sphere_gen(id++, 1.5);
  shapes_list = *(present_1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.position =
      helper::points_to_object(position_x + 1, position_y - 2, position_z + 4);
  shape.color = helper::rgb_to_object(1, 0, 1, 1);
  shapes_marker_array.push_back(shape);

  auto present_2 = shapes::triangular_pyramid_gen(id++, 0.75, 5);
  shapes_list = *(present_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.position = helper::points_to_object(
      position_x + 0.2, position_y - 1.2, position_z + 3);
  shape.color = helper::rgb_to_object(0.5, 0.2, 1, 1);
  shapes_marker_array.push_back(shape);

  auto present_3 = shapes::octagonal_pyramid_gen(id++, 0.75, 7);
  shapes_list = *(present_3->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.orientation =
      helper::point_to_quaternion(0, -helper::pi / 6, -helper::pi / 9);
  shape.pose.position = helper::points_to_object(
      position_x - 1.1, position_y + 1.4, position_z + 3);
  shape.color = helper::rgb_to_object(0.4, 1, 0.2, 1);
  shapes_marker_array.push_back(shape);

  auto present_4 = shapes::cylinder_gen(id++, 1.2, 3);
  shapes_list = *(present_4->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.position = helper::points_to_object(
      position_x - 1.5, position_y - 1.5, position_z + 3);
  shape.color = helper::rgb_to_object(0, 1, 0, 1);
  shapes_marker_array.push_back(shape);

  auto present_5 = shapes::cone_gen(id++, 0.5, 4);
  shapes_list = *(present_5->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.position = helper::points_to_object(
      position_x - 1.5, position_y + 0.2, position_z + 2.5);
  shape.color = helper::rgb_to_object(0.7, 0.1, 1, 0.5);
  shapes_marker_array.push_back(shape);

  auto present_6 = shapes::square_pyramid_gen(id++, 2, 4);
  shapes_list = *(present_6->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.position =
      helper::points_to_object(position_x, position_y + 0.2, position_z + 2.5);
  shape.color = helper::rgb_to_object(0.3, 0.5, 0.2, 1);
  shapes_marker_array.push_back(shape);

  auto present_7 = shapes::rectangular_pyramid_gen(id++, 3, 4, 2);
  shapes_list = *(present_7->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "UAV_PRESENTS";
  shape.pose.position = helper::points_to_object(
      position_x + 0.5, position_y + 2.2, position_z + 2.5);
  shape.pose.orientation =
      helper::point_to_quaternion(0, helper::pi / 6, helper::pi / 2);
  shape.color = helper::rgb_to_object(0.2, 0.4, 1, 0.8);
  shapes_marker_array.push_back(shape);

  // Get first reference for rotor.
  std::reference_wrapper<visualization_msgs::msg::Marker> reference =
      shapes_marker_array[1];
  rotors.push_back(reference);

  // Get second Reference for rotor.
  std::reference_wrapper<visualization_msgs::msg::Marker> reference2 =
      shapes_marker_array[2];
  rotors.push_back(reference2);

  // Timing reset.
  using namespace std::chrono_literals;
  for (unsigned int i = 0; i < shapes_marker_array.size(); i++) {
    shapes_marker_array[i].lifetime = rclcpp::Duration{64s};
  }

  //===========================================================================
  //
  //                        UAV Control and Callbacks
  //
  //===========================================================================
  this->velocity_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  this->velocity_->twist.linear.x = 0;
  this->velocity_->twist.linear.y = 0;
  this->velocity_->twist.linear.z = 0;
  this->velocity_->twist.angular.z = 0;
  this->velocity_->header.stamp = rclcpp::Node::now();

  // Subscribe to Joy callback.
  auto joy_callback_wrapper =
      std::bind(&Uav::joy_callback, this, std::placeholders::_1);
  this->joystick_input_ = create_subscription<sensor_msgs::msg::Joy>(
      std::string("/" + zid + "/joy"), 10, joy_callback_wrapper);

  // Set up wall timer.
  auto pose_callback_wrapper = std::bind(&Uav::pose_callback, this);
  this->timer_ = create_wall_timer(std::chrono::milliseconds{refresh_period},
                                   pose_callback_wrapper);
}

// Define joystick callback to read in joystick messages and assign them to
// parameters.
auto Uav::joy_callback(sensor_msgs::msg::Joy::UniquePtr joy_message) -> void {
  double x_axis = joy_message->axes[config_.x_axis];
  double y_axis = joy_message->axes[config_.y_axis];
  double z_plus_axis = joy_message->axes[config_.z_plus_axis];
  double z_minus_axis = joy_message->axes[config_.z_minus_axis];
  double steering_axis = joy_message->axes[config_.steering_axis];
  double trigger_deadzone = config_.trigger_deadzone;
  double joystick_deadzone = config_.joystick_deadzone;

  // Scale Input Points outside of the deadzone
  x_axis = std::round(helper::scale_with_deadzone(joystick_deadzone, x_axis));
  y_axis = std::round(helper::scale_with_deadzone(joystick_deadzone, y_axis));
  steering_axis =
      std::round(helper::scale_with_deadzone(joystick_deadzone, steering_axis));
  z_plus_axis =
      std::round(helper::scale_with_deadzone(trigger_deadzone, z_plus_axis));
  z_minus_axis =
      std::round(helper::scale_with_deadzone(trigger_deadzone, z_minus_axis));

  double scale_factor = 20;

  velocity_->twist.linear.x = scale_factor * (x_axis);
  velocity_->twist.linear.y = scale_factor * (-y_axis);
  velocity_->twist.linear.z = scale_factor * (z_minus_axis - z_plus_axis);
  velocity_->twist.angular.z =
      helper::pi / (2 * scale_factor) * (steering_axis);
  velocity_->header.stamp = rclcpp::Node::now();
}

// Define Pose Callback for converting inputted JS to pose.
auto Uav::pose_callback() -> void {
  // Connection Error if we have not communicated with the controller in 10
  // seconds
  if (velocity_->header.stamp.sec == 0) return;
  double last_message_diff =
      (rclcpp::Node::now().seconds() - velocity_->header.stamp.sec);
  if (last_message_diff >= 10) {
    std::cout << "Connection Lost ... Please Reconnect Controller" << std::endl;
    // Set the time back to zero so that we don't get message spam
    velocity_->header.stamp.sec = 0;
    return;
  }

  // Spin the rotors
  for (unsigned int i = 0; i < rotors.size(); i++) {
    auto const new_heading = shapelist_z_ / 100;
    auto const new_heading_q =
        tf2::Quaternion(tf2::Vector3(0, 0, 1), new_heading);

    tf2::Quaternion q_original;
    tf2::fromMsg((rotors[i].get()).pose.orientation, q_original);

    tf2::Quaternion q_new = new_heading_q * q_original;
    (rotors[i].get()).pose.orientation = tf2::toMsg(q_new);
  }

  // Timer is called every 100ms = 0.1s
  double dt = 0.1;

  // Change Velocity of Uav if needed
  double scale_factor = 1;

  // Move shape to appropriate location
  move_by(XAxis{dt * scale_factor * velocity_->twist.linear.x},
          YAxis{dt * scale_factor * velocity_->twist.linear.y},
          ZAxis{dt * scale_factor * velocity_->twist.linear.z});

  // Check for collisions
  move_to(XAxis{helper::bound_value(shapelist_x_, config_.side_bound,
                                    -config_.side_bound)},
          YAxis{helper::bound_value(shapelist_y_, config_.side_bound,
                                    -config_.side_bound)},
          ZAxis{helper::bound_value(shapelist_z_, config_.ceiling_bound,
                                    config_.floor_bound)});

  // Limit the Uav's movement based off config bounds
  shapelist_x_ = helper::bound_value(shapelist_x_, config_.side_bound,
                                     -config_.side_bound);
  shapelist_y_ = helper::bound_value(shapelist_y_, config_.side_bound,
                                     -config_.side_bound);
  shapelist_z_ = helper::bound_value(shapelist_z_, config_.ceiling_bound,
                                     config_.floor_bound);

  // Spin UAV based on trigger
  rotate_about_axis_to(
      ZAxis(get_orientation().get_value() + velocity_->twist.angular.z));
}
}  // namespace shapes

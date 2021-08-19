// Copyright 2019 Zhihao Zhang License MIT

#ifndef STUDENT_HELPER_HPP_
#define STUDENT_HELPER_HPP_

#include "multiple_shape_display.hpp"
#include "rclcpp/rclcpp.hpp"  // http://docs.ros2.org/dashing/api/rclcpp/
#include "single_shape_display.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

namespace helper {

auto colour_to_rgb(shapes::ColourInterface::Colour c)
    -> std_msgs::msg::ColorRGBA;
auto rgb_to_colour(std_msgs::msg::ColorRGBA c)
    -> shapes::ColourInterface::Colour;

auto range_map(double range_start, double range_end, double new_range_start,
               double new_range_end, double value) -> double;

auto scale_with_deadzone(double deadzone, double value) -> double;

auto points_to_object(double x, double y, double z)
    -> geometry_msgs::msg::Point;
auto points_to_vector3(double x, double y, double z)
    -> geometry_msgs::msg::Vector3;

auto rgb_to_object(double r, double g, double b, double a)
    -> std_msgs::msg::ColorRGBA;

auto bound_value(double value, double bound, double minus_bound) -> double;

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
auto point_to_quaternion(double yaw, double pitch, double roll)
    -> geometry_msgs::msg::Quaternion;  // yaw (Z), pitch (Y), roll (X

auto constexpr pi = 3.14159265358979323846;

// Pyramid and Prism Generation Expressions
auto constexpr CYLINDER = 64;
auto constexpr CONE = 64;
auto constexpr TRIANGULAR = 3;
auto constexpr RECTANGULAR = 4;
auto constexpr OCTAGONAL = 8;

auto constexpr RIGHT_ANGLE = pi / 4;
auto constexpr PARALLELEPIPED_ANGLE = 0;

auto constexpr joy_node_name = [](std::string const& zid) {
  return zid + "_input_node";
};

auto constexpr velocity_node_name = [](std::string const& zid) {
  return zid + "_velocity_node";
};

auto constexpr pose_node_name = [](std::string const& zid) {
  return zid + "_pose_node";
};

auto constexpr uav_node_name = [](std::string const& zid) {
  return zid + "_uav_node";
};

auto constexpr cube_list_node_name = [](std::string const& zid) {
  return zid + "_cube_list_node";
};

auto constexpr transform_node_name = [](std::string const& zid) {
  return zid + "_transform_node";
};

auto constexpr marker_node_name = [](std::string const& zid) {
  return zid + "_marker_node";
};

auto constexpr local_frame_name = [](std::string const& zid) {
  return "/" + zid + "/local_frame";
};

auto constexpr world_frame_name = [](std::string const& zid) {
  return "/" + zid + "/world_frame";
};

}  // namespace helper
#endif  // STUDENT_HELPER_HPP_

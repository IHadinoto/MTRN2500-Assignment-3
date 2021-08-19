// Copyright 2019 Zhihao Zhang License MIT

#include "student_helper.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

namespace helper {

auto colour_to_rgb(shapes::ColourInterface::Colour c)
    -> std_msgs::msg::ColorRGBA {
  std_msgs::msg::ColorRGBA rgb;

  if (c == shapes::ColourInterface::Colour::red) {
    rgb.r = 1;
    rgb.g = 0;
    rgb.b = 0;
  } else if (c == shapes::ColourInterface::Colour::yellow) {
    rgb.r = 1;
    rgb.g = 1;
    rgb.b = 0;
  } else if (c == shapes::ColourInterface::Colour::green) {
    rgb.r = 0;
    rgb.g = 1;
    rgb.b = 0;
  } else if (c == shapes::ColourInterface::Colour::blue) {
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 1;
  } else if (c == shapes::ColourInterface::Colour::black) {
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 0;
  } else {
    // white
    rgb.r = 1;
    rgb.g = 1;
    rgb.b = 1;
  }
  rgb.a = 1;
  return rgb;
}

auto rgb_to_colour(std_msgs::msg::ColorRGBA c)
    -> shapes::ColourInterface::Colour {
  if (c.r == 1 && c.g == 0 && c.b == 0) {
    return shapes::ColourInterface::Colour::red;
  } else if (c.r == 1 && c.g == 1 && c.b == 0) {
    return shapes::ColourInterface::Colour::yellow;
  } else if (c.r == 0 && c.g == 1 && c.b == 0) {
    return shapes::ColourInterface::Colour::green;
  } else if (c.r == 0 && c.g == 0 && c.b == 1) {
    return shapes::ColourInterface::Colour::blue;
  } else if (c.r == 0 && c.g == 0 && c.b == 0) {
    return shapes::ColourInterface::Colour::black;
  } else {
    return shapes::ColourInterface::Colour::white;
  }
}

// Ease of use creation of Point object
auto points_to_object(double x, double y, double z)
    -> geometry_msgs::msg::Point {
  geometry_msgs::msg::Point point_object;
  point_object.x = x;
  point_object.y = y;
  point_object.z = z;
  return point_object;
}

// Ease of use creation of Vector3 object (USED FOR SCALE)
auto points_to_vector3(double x, double y, double z)
    -> geometry_msgs::msg::Vector3 {
  geometry_msgs::msg::Vector3 vector3_object;
  vector3_object.x = x;
  vector3_object.y = y;
  vector3_object.z = z;
  return vector3_object;
}

// Ease of use creation of ColorRGBA object
auto rgb_to_object(double r, double g, double b, double a)
    -> std_msgs::msg::ColorRGBA {
  std_msgs::msg::ColorRGBA rgb_object;
  rgb_object.r = r;
  rgb_object.g = g;
  rgb_object.b = b;
  rgb_object.a = a;
  return rgb_object;
}

// Bounds a value in an absolute value range set by bound
auto bound_value(double value, double bound, double minus_bound) -> double {
  if (value > bound) return bound;
  if (value < minus_bound) return minus_bound;
  return value;
}

// Maps Value from range -> new_range
auto range_map(double range_start, double range_end, double new_range_start,
               double new_range_end, double value) -> double {
  // Handle Zero Division
  if (range_end - range_start == 0) return 0;

  // Derived formula for linear map between 2 ranges
  return 1.0 * (value - range_start) / (range_end - range_start) *
             (new_range_end - new_range_start) +
         new_range_start;
}

// Removes points in the deadzone
auto scale_with_deadzone(double deadzone, double value) -> double {
  // Values in the Deadzone are 0
  if (fabs(deadzone) > fabs(value)) return 0;

  // Other Values are scaled appropriately
  if (value < 0) return range_map(-deadzone, -1, 0, -1, value);
  return range_map(deadzone, 1, 0, 1, value);
}

auto point_to_quaternion(double yaw, double pitch, double roll)
    -> geometry_msgs::msg::Quaternion  // yaw (Z), pitch (Y), roll (X)
{
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

}  // namespace helper

// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "sphere.hpp"

#include "rclcpp/rclcpp.hpp"  // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes {

Sphere::Sphere(int id) : ShapeBase(id) {
  auto& shapes_list = *shapes_list_ptr_;
  auto& shape = shapes_list[0];

  // Type of marker we want to display
  shape.type = visualization_msgs::msg::Marker::SPHERE;
}
}  // namespace shapes

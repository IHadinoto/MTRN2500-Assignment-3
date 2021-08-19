// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "prism.hpp"

#include "rclcpp/rclcpp.hpp"  // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace shapes {
/**
 * \brief Example of how Triangle class may be implemented, the design may not
 *be the most suitable. Trivial functionalities are not implemented.
 **/

Prism::Prism(int id, int number_of_sides, double input_angle, double radius,
             double height)
    : ShapeBase(id) {
  // Get a ref to the vector of marker for ease of use
  auto& shapes_list = *shapes_list_ptr_;
  // get a ref to the new marker.
  auto& shape = shapes_list[0];

  shape = create_prism(id, number_of_sides, input_angle, radius, height);
}
// Get a ref to the vector of marker for ease of use
auto Prism::create_prism(int id, int number_of_sides, double input_angle,
                         double radius, double height)
    -> visualization_msgs::msg::Marker {
  auto prism =
      create_shape_base(id, visualization_msgs::msg::Marker::TRIANGLE_LIST);

  double angle =
      (2 * helper::pi) / number_of_sides;  //((number_of_sides - 2) *
                                           //helper::pi/2)/number_of_sides;;
  double angle_sum = 0;

  for (int i = 0; i < number_of_sides; i++) {
    double x1 = radius * cos(angle_sum - angle + input_angle);
    double x2 = radius * cos(angle_sum + input_angle);
    double y1 = radius * sin(angle_sum - angle + input_angle);
    double y2 = radius * sin(angle_sum + input_angle);

    auto p1 = helper::points_to_object(0, 0, 0);
    auto p2 = helper::points_to_object(x2, y2, 0);
    auto p3 = helper::points_to_object(x1, y1, 0);
    auto p4 = helper::points_to_object(0, 0, height);
    auto p5 = helper::points_to_object(x2, y2, height);
    auto p6 = helper::points_to_object(x1, y1, height);

    // Prism Base
    prism.points.push_back(p2);
    prism.points.push_back(p3);
    prism.points.push_back(p1);

    // Prism Top
    prism.points.push_back(p4);
    prism.points.push_back(p6);
    prism.points.push_back(p5);

    // Prism Sides
    prism.points.push_back(p5);
    prism.points.push_back(p3);
    prism.points.push_back(p2);

    prism.points.push_back(p5);
    prism.points.push_back(p6);
    prism.points.push_back(p3);

    angle_sum = angle_sum + angle;
  }

  return prism;
}
}  // namespace shapes

// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "pyramid.hpp"

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

Pyramid::Pyramid(int id, int number_of_sides, double input_angle, double radius,
                 double height)
    : ShapeBase(id) {
  // Get a ref to the vector of marker for ease of use
  auto& shapes_list = *shapes_list_ptr_;
  // get a ref to the new marker.
  auto& shape = shapes_list[0];

  shape = create_pyramid(id, number_of_sides, input_angle, radius, height);
}
auto Pyramid::create_pyramid(int id, int number_of_sides, double input_angle,
                             double radius, double height)
    -> visualization_msgs::msg::Marker {
  auto pyramid =
      create_shape_base(id, visualization_msgs::msg::Marker::TRIANGLE_LIST);

  // Implement loop to create pyramid
  // Introduce variables
  double angle =
      (2 * helper::pi) / number_of_sides;  //((number_of_sides - 2) *
                                           //helper::pi/2)/number_of_sides;;

  double x1 = 0;
  double x2 = 0;
  double y1 = 0;
  double y2 = 0;

  double angle_sum = 0;

  for (int x = 0; x < number_of_sides; x++) {
    x1 = radius * cos(angle_sum - angle + input_angle);
    x2 = radius * cos(angle_sum + input_angle);
    y1 = radius * sin(angle_sum - angle + input_angle);
    y2 = radius * sin(angle_sum + input_angle);

    auto p1 = helper::points_to_object(0, 0, 0);
    auto p2 = helper::points_to_object(x2, y2, 0);
    auto p3 = helper::points_to_object(x1, y1, 0);
    auto p4 = helper::points_to_object(0, 0, height);

    // Pyramid Base
    pyramid.points.push_back(p2);
    pyramid.points.push_back(p1);
    pyramid.points.push_back(p3);

    // Pyramid Sides
    pyramid.points.push_back(p4);
    pyramid.points.push_back(p3);
    pyramid.points.push_back(p2);

    angle_sum = angle_sum + angle;
  }
  return pyramid;
}

}  // namespace shapes

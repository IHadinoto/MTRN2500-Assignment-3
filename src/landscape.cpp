// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "landscape.hpp"

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

Landscape::Landscape(
    int id,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> landscape_list_ptr)
    : ShapeBaseMulti(landscape_list_ptr) {
  // Get a ref to the vector of marker for ease of use
  auto& shapes_marker_array = shapes_list_list_ptr_->markers;

  //===========================================================================
  //                             SHAPES CREATION
  //===========================================================================

  int position_x = 0;
  int position_y = 0;
  int position_z = 0;

  // Flat Plane
  auto flat = flat_plane_gen(id++, 1, 1);
  auto& shapes_list = *(flat->get_display_markers());
  auto& shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z);
  shape.color = helper::rgb_to_object(0, 1, 0, 1);
  shape.scale = helper::points_to_vector3(300, 300, 0.01);
  shapes_marker_array.push_back(shape);

  auto log = shapes::rectangular_prism_gen(id++, 60, 60, 4);
  shapes_list = *(log->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shapes_marker_array.push_back(shape);

  auto log2 = shapes::octagonal_prism_gen(id++, 2, 20);
  shapes_list = *(log2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 4);
  shape.color = helper::rgb_to_object(0.5, 0.25, 0, 1);
  shapes_marker_array.push_back(shape);

  auto leaf1 = shapes::octagonal_pyramid_gen(id++, 30, 20);
  shapes_list = *(leaf1->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 20);
  shape.color = helper::rgb_to_object(0.5, 1, 0.5, 1);
  // shape.scale = helper::points_to_vector3(60,60,4);
  shapes_marker_array.push_back(shape);

  auto leaf2 = shapes::rectangular_pyramid_gen(id++, 40, 30, 20);
  shapes_list = *(leaf2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 40);
  shape.color = helper::rgb_to_object(0.5, 1, 0.5, 1);
  shapes_marker_array.push_back(shape);

  auto leaf3 = shapes::triangular_pyramid_gen(id++, 10, 20);
  shapes_list = *(leaf3->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 60);
  shape.color = helper::rgb_to_object(0.5, 1, 0.5, 1);
  shapes_marker_array.push_back(shape);

  auto star = shapes::rectangular_pyramid_gen(id++, 5, 5, 15);
  shapes_list = *(star->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 80);
  shape.color = helper::rgb_to_object(1, 1, 0, 1);
  shapes_marker_array.push_back(shape);

  auto house1_base = shapes::cylinder_gen(id++, 10, 20);
  shapes_list = *(house1_base->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x + 80, position_y - 20, position_z);
  shape.color = helper::rgb_to_object(0.2, 1, 1, 1);
  shapes_marker_array.push_back(shape);

  auto house1_roof = shapes::cone_gen(id++, 10, 20);
  shapes_list = *(house1_roof->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position = helper::points_to_object(
      position_x + 80, position_y - 20, position_z + 20);
  shape.color = helper::rgb_to_object(1, 0.1, 0, 5);
  shapes_marker_array.push_back(shape);

  auto house2_base = shapes::cube_gen(id++, 20);
  shapes_list = *(house2_base->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x - 70, position_y - 60, position_z);
  shape.color = helper::rgb_to_object(1, 0.4, 0, 1);
  shapes_marker_array.push_back(shape);

  auto house2_roof = shapes::sphere_gen(id++, 20);
  shapes_list = *(house2_roof->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position = helper::points_to_object(
      position_x - 70, position_y - 60, position_z + 10);
  shape.color = helper::rgb_to_object(0.8, 0.8, 0, 5);
  shapes_marker_array.push_back(shape);

  auto house3_base = shapes::triangular_prism_gen(id++, 20, 20);
  shapes_list = *(house3_base->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x - 40, position_y + 60, position_z);
  shape.color = helper::rgb_to_object(0, 0.4, 1, 1);
  shapes_marker_array.push_back(shape);

  auto house3_roof = shapes::parallelepiped_gen(id++, 40, 30, 10);
  shapes_list = *(house3_roof->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position = helper::points_to_object(
      position_x - 40, position_y + 60, position_z + 20);
  shape.color = helper::rgb_to_object(1, 1, 0, 5);
  shapes_marker_array.push_back(shape);

  auto snow = shapes::cube_gen(id++, 1);
  shapes_list = *(snow->get_display_markers());
  shape = shapes_list[0];
  shape.scale = helper::points_to_vector3(300, 300, 10);
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 0.1);
  shape.color = helper::rgb_to_object(1, 1, 1, 0.4);
  shapes_marker_array.push_back(shape);

  auto beacon = shapes::cone_gen(id++, 1, 1);
  shapes_list = *(beacon->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z + 5);
  shape.color = helper::rgb_to_object(1, 0, 1, 1);
  shape.scale = helper::points_to_vector3(0.4, 0.4, 1);
  shapes_marker_array.push_back(shape);

  auto flat_2 = flat_plane_gen(id++, 6, 6);
  shapes_list = *(flat_2->get_display_markers());
  shape = shapes_list[0];
  shape.ns = "LANDSCAPE";
  shape.pose.position =
      helper::points_to_object(position_x, position_y, position_z);
  shape.color = helper::rgb_to_object(1, 0, 1, 0.2);
  shapes_marker_array.push_back(shape);

  using namespace std::chrono_literals;
  for (unsigned int i = 0; i < shapes_marker_array.size(); i++) {
    shapes_marker_array[i].lifetime = rclcpp::Duration{60s};
  }
}
}  // namespace shapes

// Copyright 2019 Zhihao Zhang License MIT

#include "multiple_shape_display.hpp"

#include "student_helper.hpp"

#include <memory>
#include <string>
#include <vector>

namespace {
auto constexpr marker_topic = [](std::string const& zid) {
  return "/" + zid + "/marker";
};
}  // namespace

namespace display {
MultipleShapeDisplay::MultipleShapeDisplay(
    std::string const& node_name,
    std::chrono::milliseconds const refresh_period,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_array)
    : rclcpp::Node{node_name},
      marker_publisher_{create_publisher<visualization_msgs::msg::MarkerArray>(
          "/z0000000/marker_array", 10)}  // Set the publisher name
      ,
      timer_{create_wall_timer(
          refresh_period, [this]() -> void { marker_publisher_callback(); })}
      // Periodically publish
      ,
      marker_array_{marker_array} {}

auto MultipleShapeDisplay::display_object_imple(
    std::shared_ptr<shapes::DisplayableInterface> const display_object)
    -> void  // Displayable Interface from interfaces.hpp
{
  object_to_display_ =
      display_object;  // set the shared pointer to a different name
}

// ReSharper disable once CppMemberFunctionMayBeConst
auto MultipleShapeDisplay::marker_publisher_callback() -> void {
  auto const shapes_list = marker_array_;  // Get a list of the shapes
  marker_publisher_->publish(*shapes_list);
}
}  // namespace display

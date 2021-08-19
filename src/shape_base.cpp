// Copyright 2019 Zhihao Zhang License MIT
#include "shape_base.hpp"

#include "rclcpp/rclcpp.hpp"  // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <math.h>
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

ShapeBase::ShapeBase(int id)
    : heading_{ZAxis{0.0}},
      parent_frame_name_{helper::local_frame_name("z0000000")},
      shapes_list_ptr_{
          std::make_shared<std::vector<visualization_msgs::msg::Marker>>()} {
  // Get a ref to the vector of marker for ease of use
  auto& shapes_list = *shapes_list_ptr_;
  // create a new marker
  shapes_list.emplace_back();
  // get a ref to the new marker.
  auto& shape = shapes_list[0];

  shape = create_shape_base(id, visualization_msgs::msg::Marker::CUBE);
}

auto ShapeBase::resize_imple(AllAxis const new_size) -> void {
  shapes_list_ptr_->at(0).scale.x = new_size.get_value();
  shapes_list_ptr_->at(0).scale.y = new_size.get_value();
  shapes_list_ptr_->at(0).scale.z = new_size.get_value();
}

auto ShapeBase::rescale_imple(AnyAxis const factor) -> void {
  shapes_list_ptr_->at(0).scale.x *= factor.get_value();
  shapes_list_ptr_->at(0).scale.y *= factor.get_value();
  shapes_list_ptr_->at(0).scale.z *= factor.get_value();
}

auto ShapeBase::set_colour_imple(Colour c) -> void {
  shapes_list_ptr_->at(0).color = helper::colour_to_rgb(c);
}

auto ShapeBase::get_colour_imple() const -> Colour {
  return helper::rgb_to_colour(shapes_list_ptr_->at(0).color);
}

auto ShapeBase::set_parent_frame_name_imple(std::string frame_name) -> void {
  parent_frame_name_ = std::move(frame_name);
}

auto ShapeBase::get_location_imple() const -> std::tuple<XAxis, YAxis, ZAxis> {
  return std::tuple{XAxis{shapes_list_ptr_->at(0).pose.position.x},
                    YAxis{shapes_list_ptr_->at(0).pose.position.y},
                    ZAxis{shapes_list_ptr_->at(0).pose.position.z}};
}

auto ShapeBase::move_to_imple(XAxis const x) -> void {
  shapes_list_ptr_->at(0).pose.position.x = x.get_value();
}

auto ShapeBase::move_to_imple(YAxis const y) -> void {
  shapes_list_ptr_->at(0).pose.position.y = y.get_value();
}

auto ShapeBase::move_to_imple(ZAxis const z) -> void {
  shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

/**
 * \brief Move the shape to a new location.
 * \param x new x location
 * \param y new y location
 * \param z new z location
 */
auto ShapeBase::move_to_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void {
  shapes_list_ptr_->at(0).pose.position.x = x.get_value();
  shapes_list_ptr_->at(0).pose.position.y = y.get_value();
  shapes_list_ptr_->at(0).pose.position.z = z.get_value();
}

auto ShapeBase::move_by_imple(XAxis const x) -> void {
  shapes_list_ptr_->at(0).pose.position.x += x.get_value();
}

auto ShapeBase::move_by_imple(YAxis const y) -> void {
  shapes_list_ptr_->at(0).pose.position.y += y.get_value();
}

auto ShapeBase::move_by_imple(ZAxis const z) -> void {
  shapes_list_ptr_->at(0).pose.position.z += z.get_value();
}

auto ShapeBase::move_by_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void {
  shapes_list_ptr_->at(0).pose.position.x += x.get_value();
  shapes_list_ptr_->at(0).pose.position.y += y.get_value();
  shapes_list_ptr_->at(0).pose.position.z += z.get_value();
}

/**
 * \brief Return marker message for displaying the shape
 * \return shape marker message
 */
auto ShapeBase::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> {
  return shapes_list_ptr_;
}

auto ShapeBase::create_shape_base(int id, std::int32_t type)
    -> visualization_msgs::msg::Marker {
  visualization_msgs::msg::Marker shape;

  // Parent frame name
  shape.header.frame_id = helper::world_frame_name("z0000000");
  // body.header.stamp

  // namespace the marker will be in
  shape.ns = "";
  // Used to identify which marker we are adding/modifying/deleting
  // Must be unique between shape objects.
  shape.id = id;

  // Default Shape Type
  shape.type = type;
  // Add, modify or delete.
  shape.action = visualization_msgs::msg::Marker::ADD;

  // Position
  shape.pose.position.x = 0;
  shape.pose.position.y = 0;
  shape.pose.position.z = 0;

  // Orientation in quaternion. Check transform marker in assignment 2
  // for how to manipulate it.
  shape.pose.orientation.x = 0;
  shape.pose.orientation.y = 0;
  shape.pose.orientation.z = 0;
  shape.pose.orientation.w = 1;

  // Scale change the dimension of the sides.
  shape.scale.x = 1.0;
  shape.scale.y = 1.0;
  shape.scale.z = 1.0;

  // colour red, green, blue, alpha (transparency)
  shape.color.r = 1.0;
  shape.color.g = 1.0;
  shape.color.b = 0.0;
  shape.color.a = 1.0;

  // body.colors.emplace_back();
  using namespace std::chrono_literals;
  shape.lifetime =
      rclcpp::Duration{1s};  // How long our marker message is valid for

  return shape;
}

auto ShapeBase::rotate_about_axis_to_imple(ZAxis radians) -> void {
  auto const new_heading = radians.get_value() - heading_.get_value();
  auto const new_heading_q =
      tf2::Quaternion(tf2::Vector3(0, 0, 1), new_heading);

  tf2::Quaternion q_original;
  tf2::fromMsg(shapes_list_ptr_->at(0).pose.orientation, q_original);

  tf2::Quaternion q_new = new_heading_q * q_original;
  shapes_list_ptr_->at(0).pose.orientation = tf2::toMsg(q_new);
  heading_.set_value(shapes_list_ptr_->at(0).pose.orientation.z);
}

auto ShapeBase::get_orientation_imple() const -> ZAxis {
  return heading_;
}  // namespace shapes
}  // namespace shapes

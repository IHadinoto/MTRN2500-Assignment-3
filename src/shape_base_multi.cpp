// Copyright 2019 Zhihao Zhang License MIT
#include "shape_base_multi.hpp"

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

ShapeBaseMulti::ShapeBaseMulti(
    std::shared_ptr<visualization_msgs::msg::MarkerArray> vector_pointer)
    : heading_{ZAxis{0.0}},
      parent_frame_name_{helper::local_frame_name("z0000000")},
      shapes_list_list_ptr_{vector_pointer} {}

auto ShapeBaseMulti::resize_imple(AllAxis const new_size) -> void {
  double scale_ratio = new_size.get_value() / shapelist_scale_;
  shapelist_scale_ = new_size.get_value();
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].scale.x *= scale_ratio;
    shapes_list[i].scale.y *= scale_ratio;
    shapes_list[i].scale.z *= scale_ratio;

    shapes_list[i].pose.position.x =
        (shapes_list[i].pose.position.x - shapelist_x_) * scale_ratio +
        shapelist_x_;
    shapes_list[i].pose.position.y =
        (shapes_list[i].pose.position.y - shapelist_y_) * scale_ratio +
        shapelist_y_;
    shapes_list[i].pose.position.z =
        (shapes_list[i].pose.position.z - shapelist_z_) * scale_ratio +
        shapelist_z_;
  }
}

auto ShapeBaseMulti::rescale_imple(AnyAxis const factor) -> void {
  shapelist_scale_ *= factor.get_value();
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].scale.x *= shapelist_scale_;
    shapes_list[i].scale.y *= shapelist_scale_;
    shapes_list[i].scale.z *= shapelist_scale_;

    shapes_list[i].pose.position.x =
        (shapes_list[i].pose.position.x - shapelist_x_) * shapelist_scale_ +
        shapelist_x_;
    shapes_list[i].pose.position.y =
        (shapes_list[i].pose.position.y - shapelist_y_) * shapelist_scale_ +
        shapelist_y_;
    shapes_list[i].pose.position.z =
        (shapes_list[i].pose.position.z - shapelist_z_) * shapelist_scale_ +
        shapelist_z_;
  }
}

auto ShapeBaseMulti::set_colour_imple(Colour c) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].color = helper::colour_to_rgb(c);
  }
}

auto ShapeBaseMulti::get_colour_imple() const -> Colour {
  // useless function, placeholder
  return helper::rgb_to_colour(helper::rgb_to_object(0, 0, 0, 0));
}

auto ShapeBaseMulti::set_parent_frame_name_imple(std::string frame_name)
    -> void {
  std::cout << frame_name << std::endl;
  // useless function, placeholder
}

auto ShapeBaseMulti::get_location_imple() const
    -> std::tuple<XAxis, YAxis, ZAxis> {
  return std::tuple{XAxis{shapelist_x_}, YAxis{shapelist_y_},
                    ZAxis{shapelist_z_}};
}

auto ShapeBaseMulti::move_to_imple(XAxis const x) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.x += x.get_value() - shapelist_x_;
  }

  shapelist_x_ = x.get_value();
}

auto ShapeBaseMulti::move_to_imple(YAxis const y) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.y += y.get_value() - shapelist_y_;
  }

  shapelist_y_ = y.get_value();
}

auto ShapeBaseMulti::move_to_imple(ZAxis const z) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.z += z.get_value() - shapelist_z_;
  }

  shapelist_z_ = z.get_value();
}

/**
 * \brief Move the shape to a new location.
 * \param x new x location
 * \param y new y location
 * \param z new z location
 */
auto ShapeBaseMulti::move_to_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.x += x.get_value() - shapelist_x_;
    shapes_list[i].pose.position.y += y.get_value() - shapelist_y_;
    shapes_list[i].pose.position.z += z.get_value() - shapelist_z_;
  }

  shapelist_x_ = x.get_value();
  shapelist_y_ = y.get_value();
  shapelist_z_ = z.get_value();
}

auto ShapeBaseMulti::move_by_imple(XAxis const x) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.x += x.get_value();
  }
  shapelist_x_ += x.get_value();
}

auto ShapeBaseMulti::move_by_imple(YAxis const y) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.y += y.get_value();
  }
  shapelist_y_ += y.get_value();
}

auto ShapeBaseMulti::move_by_imple(ZAxis const z) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.z += z.get_value();
  }
  shapelist_z_ += z.get_value();
}

auto ShapeBaseMulti::move_by_imple(XAxis const x, YAxis const y, ZAxis const z)
    -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    shapes_list[i].pose.position.x += x.get_value();
    shapes_list[i].pose.position.y += y.get_value();
    shapes_list[i].pose.position.z += z.get_value();
  }
  shapelist_x_ += x.get_value();
  shapelist_y_ += y.get_value();
  shapelist_z_ += z.get_value();
}

/**
 * \brief Return marker message for displaying the shape
 * \return shape marker message
 */
auto ShapeBaseMulti::get_display_markers_imple()
    -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> {
  return NULL;
}

auto ShapeBaseMulti::get_display_list()
    -> std::shared_ptr<visualization_msgs::msg::MarkerArray> {
  return shapes_list_list_ptr_;
}

auto ShapeBaseMulti::rotate_about_axis_to_imple(ZAxis radians) -> void {
  auto& shapes_list = shapes_list_list_ptr_->markers;
  if (!shapes_list.size()) return;

  auto const new_heading = radians.get_value() - shapelist_heading_.get_value();
  auto const new_heading_q =
      tf2::Quaternion(tf2::Vector3(0, 0, 1), new_heading);

  for (unsigned int i = 0; i < shapes_list.size(); i++) {
    // Rotate object normally
    tf2::Quaternion q_original;
    tf2::fromMsg(shapes_list[i].pose.orientation, q_original);

    tf2::Quaternion q_new = new_heading_q * q_original;
    shapes_list[i].pose.orientation = tf2::toMsg(q_new);

    // Change object's position
    // Recenter around origin
    double rel_x = shapes_list[i].pose.position.x - shapelist_x_;
    double rel_y = shapes_list[i].pose.position.y - shapelist_y_;

    // Calculate Delta X
    double delta_x = (rel_x)*cos(new_heading) - (rel_y)*sin(new_heading);
    double delta_y = (rel_x)*sin(new_heading) + (rel_y)*cos(new_heading);

    // Rereference coordinate frame
    shapes_list[i].pose.position.x = delta_x + shapelist_x_;
    shapes_list[i].pose.position.y = delta_y + shapelist_y_;
  }

  shapelist_heading_.set_value(shapes_list[0].pose.orientation.z);
}

auto ShapeBaseMulti::get_orientation_imple() const -> ZAxis {
  return shapelist_heading_;
}
}  // namespace shapes

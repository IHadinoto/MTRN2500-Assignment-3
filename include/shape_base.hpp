// Copyright 2019 Zhihao Zhang License MIT

#ifndef SHAPE_BASE_HPP_
#define SHAPE_BASE_HPP_

#include "interfaces.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes {
// ReSharper disable once CppClassCanBeFinal
class ShapeBase : public ShapeCommonInterface {
 public:
  explicit ShapeBase(int id);

 protected:
  ZAxis heading_;
  std::string parent_frame_name_;
  std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
      shapes_list_ptr_;

  auto resize_imple(AllAxis new_size) -> void override;
  auto rescale_imple(AnyAxis factor) -> void override;
  auto set_colour_imple(Colour c) -> void override;
  [[nodiscard]] auto get_colour_imple() const -> Colour override;

  auto set_parent_frame_name_imple(std::string frame_name) -> void override;
  [[nodiscard]] auto get_location_imple() const
      -> std::tuple<XAxis, YAxis, ZAxis> override;

  auto move_to_imple(XAxis) -> void override;
  auto move_to_imple(YAxis) -> void override;
  auto move_to_imple(ZAxis) -> void override;
  auto move_to_imple(XAxis, YAxis, ZAxis) -> void override;

  auto move_by_imple(XAxis) -> void override;
  auto move_by_imple(YAxis) -> void override;
  auto move_by_imple(ZAxis) -> void override;

  auto move_by_imple(XAxis, YAxis, ZAxis) -> void override;

  auto get_display_markers_imple()
      -> std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> override;

  auto rotate_about_axis_to_imple(ZAxis radians) -> void override;

  [[nodiscard]] auto get_orientation_imple() const -> ZAxis override;

  auto create_shape_base(int id, std::int32_t type)
      -> visualization_msgs::msg::Marker;
};
}  // namespace shapes
#endif  // SHAPE_BASE_HPP_

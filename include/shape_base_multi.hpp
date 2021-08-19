// Copyright 2019 Zhihao Zhang License MIT

#ifndef SHAPE_BASE_MULTI_HPP_
#define SHAPE_BASE_MULTI_HPP_

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
class ShapeBaseMulti : public ShapeCommonInterface {
 public:
  explicit ShapeBaseMulti(
      std::shared_ptr<visualization_msgs::msg::MarkerArray> vector_pointer);
  auto get_display_list()
      -> std::shared_ptr<visualization_msgs::msg::MarkerArray>;

 protected:
  ZAxis heading_;
  std::string parent_frame_name_;
  std::shared_ptr<visualization_msgs::msg::MarkerArray> shapes_list_list_ptr_;

  double shapelist_scale_ = 1;
  double shapelist_x_ = 0;
  double shapelist_y_ = 0;
  double shapelist_z_ = 0;

  ZAxis shapelist_heading_ = ZAxis{0};

  // Required Imple Functions
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
};
}  // namespace shapes
#endif  // SHAPE_BASE_HPP_

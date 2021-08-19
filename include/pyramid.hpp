// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#ifndef PYRAMID_HPP_
#define PYRAMID_HPP_

#include "interfaces.hpp"
#include "shape_base.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes {
// ReSharper disable once CppClassCanBeFinal
class Pyramid : public ShapeBase {
 public:
  explicit Pyramid(int id, int number_of_sides, double input_angle,
                   double radius, double height);

 protected:
  auto create_pyramid(int id, int number_of_sides, double input_angle,
                      double radius, double height)
      -> visualization_msgs::msg::Marker;
};
}  // namespace shapes
#endif  // PYRAMID_HPP_

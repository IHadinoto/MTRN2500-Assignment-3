// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#ifndef CUBE_HPP_
#define CUBE_HPP_

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
class Cube : public ShapeBase {
 public:
  explicit Cube(int id);
};
}  // namespace shapes
#endif  // CUBE_HPP_

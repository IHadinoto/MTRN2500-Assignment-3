#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cube.hpp"
#include "landscape.hpp"
#include "prism.hpp"
#include "pyramid.hpp"
#include "sphere.hpp"

#include "shape_generator.hpp"
#include "student_helper.hpp"

namespace shapes {
// Prism Family
auto rectangular_prism_gen(int id, double x, double y, double z)
    -> std::shared_ptr<shapes::Prism> {
  auto prism = std::make_shared<shapes::Prism>(id, helper::RECTANGULAR,
                                               helper::RIGHT_ANGLE, 0.5, 1);
  auto& shapes_list = *(prism->get_display_markers());
  auto& shape = shapes_list[0];

  shape.scale.x = x;
  shape.scale.y = y;
  shape.scale.z = z;

  return prism;
}
auto triangular_prism_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Prism> {
  auto prism = std::make_shared<shapes::Prism>(
      id, helper::TRIANGULAR, helper::RIGHT_ANGLE, radius, height);
  return prism;
}
auto octagonal_prism_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Prism> {
  auto prism = std::make_shared<shapes::Prism>(
      id, helper::OCTAGONAL, helper::RIGHT_ANGLE, radius, height);
  return prism;
}
auto cylinder_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Prism> {
  auto prism = std::make_shared<shapes::Prism>(
      id, helper::CYLINDER, helper::RIGHT_ANGLE, radius, height);
  return prism;
}
auto parallelepiped_gen(int id, double diag1, double diag2, double height)
    -> std::shared_ptr<shapes::Prism> {
  auto prism = std::make_shared<shapes::Prism>(
      id, helper::RECTANGULAR, helper::PARALLELEPIPED_ANGLE, 0.5, 1);

  auto& shapes_list = *(prism->get_display_markers());
  auto& shape = shapes_list[0];

  shape.scale.x = diag1;
  shape.scale.y = diag2;
  shape.scale.z = height;

  return prism;
}

// Pyramid Family
auto rectangular_pyramid_gen(int id, double x, double y, double height)
    -> std::shared_ptr<shapes::Pyramid> {
  auto pyramid = std::make_shared<shapes::Pyramid>(id, helper::RECTANGULAR,
                                                   helper::RIGHT_ANGLE, 0.5, 1);
  auto& shapes_list = *(pyramid->get_display_markers());
  auto& shape = shapes_list[0];

  shape.scale.x = x;
  shape.scale.y = y;
  shape.scale.z = height;

  return pyramid;
}

auto triangular_pyramid_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid> {
  auto pyramid = std::make_shared<shapes::Pyramid>(
      id, helper::TRIANGULAR, helper::RIGHT_ANGLE, radius, height);
  return pyramid;
}
auto square_pyramid_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid> {
  auto pyramid = std::make_shared<shapes::Pyramid>(
      id, helper::RECTANGULAR, helper::RIGHT_ANGLE, radius, height);
  return pyramid;
}
auto octagonal_pyramid_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid> {
  auto pyramid = std::make_shared<shapes::Pyramid>(
      id, helper::OCTAGONAL, helper::RIGHT_ANGLE, radius, height);
  return pyramid;
}
auto cone_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid> {
  auto pyramid = std::make_shared<shapes::Pyramid>(
      id, helper::CONE, helper::RIGHT_ANGLE, radius, height);
  return pyramid;
}

// Sphere
auto sphere_gen(int id, double radius) -> std::shared_ptr<shapes::Sphere> {
  auto sphere = std::make_shared<shapes::Sphere>(id);
  auto& shapes_list = *(sphere->get_display_markers());
  auto& shape = shapes_list[0];

  shape.scale.x = radius;
  shape.scale.y = radius;
  shape.scale.z = radius;

  return sphere;
}

// Cube Family
auto cube_gen(int id, double radius) -> std::shared_ptr<shapes::Cube> {
  auto const cube = std::make_shared<shapes::Cube>(id);
  auto& shapes_list = *(cube->get_display_markers());
  auto& shape = shapes_list[0];

  shape.scale.x = radius;
  shape.scale.y = radius;
  shape.scale.z = radius;

  return cube;
}
auto flat_plane_gen(int id, double length, double width)
    -> std::shared_ptr<shapes::Cube> {
  auto cube = std::make_shared<shapes::Cube>(id);
  auto& shapes_list = *(cube->get_display_markers());
  auto& shape = shapes_list[0];

  shape.scale.x = length;
  shape.scale.y = width;
  shape.scale.z = 0.1;

  return cube;
}

}  // namespace shapes

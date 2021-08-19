#ifndef SHAPE_GENERATOR_HPP_
#define SHAPE_GENERATOR_HPP_

#include "cube.hpp"
#include "prism.hpp"
#include "pyramid.hpp"
#include "sphere.hpp"

namespace shapes {
// Prism Family
auto rectangular_prism_gen(int id, double x, double y, double z)
    -> std::shared_ptr<shapes::Prism>;
auto triangular_prism_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Prism>;
auto octagonal_prism_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Prism>;
auto cylinder_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Prism>;
auto parallelepiped_gen(int id, double diag1, double diag2, double height)
    -> std::shared_ptr<shapes::Prism>;

// Cone Family
auto rectangular_pyramid_gen(int id, double x, double y, double height)
    -> std::shared_ptr<shapes::Pyramid>;
auto triangular_pyramid_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid>;
auto square_pyramid_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid>;
auto octagonal_pyramid_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid>;
auto cone_gen(int id, double radius, double height)
    -> std::shared_ptr<shapes::Pyramid>;

// Sphere
auto sphere_gen(int id, double radius) -> std::shared_ptr<shapes::Sphere>;

// Cube Family
auto cube_gen(int id, double radius) -> std::shared_ptr<shapes::Cube>;
auto flat_plane_gen(int id, double length, double width)
    -> std::shared_ptr<shapes::Cube>;

}  // namespace shapes
#endif

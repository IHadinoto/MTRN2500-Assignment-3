// SPDX-License-Identifier: MIT
/**
 *  \brief     Assignment 3 starter program
 *  \details   Program will show a sphere in RVIZ2
 *  \author    Zhihao Zhang
 *  \version   0.11.15
 *  \date      Nov 2019
 *  \copyright MIT
 **/

// Modified by Dylan Sanusi-Goh and Joshua Behar
// UNSW Sydney - MTRN2500 Assignment 3 2019 T3

#include "cube_list_markerarray.hpp"
#include "landscape.hpp"
#include "shape_generator.hpp"
#include "uav.hpp"

#include "config_parser.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
#include "multiple_shape_display.hpp"
#include "rclcpp/rclcpp.hpp"  // http://docs.ros2.org/dashing/api/rclcpp/
#include "single_shape_display.hpp"
#include "student_helper.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>  // chrono_literals, https://en.cppreference.com/w/cpp/header/chrono
#include <fstream>  // ifstream, https://en.cppreference.com/w/cpp/header/fstream
#include <iostream>  // cout, https://en.cppreference.com/w/cpp/header/iostream
#include <memory>  // make_shared, https://en.cppreference.com/w/cpp/header/memory
#include <string>  // string, https://en.cppreference.com/w/cpp/header/string
#include <vector>  // vector, https://en.cppreference.com/w/cpp/header/vector

// Main construction of all single and multi shapes.
auto main(int argc, char* argv[]) -> int {
  using namespace std::chrono_literals;

  try {
    // Read config from std::in.
    std::ifstream config_file;
    if (argc > 1)
      config_file.open(argv[1]);
    else {
      std::cout
          << "File not found, please specify file as a commandline argument"
          << std::endl;
      exit(EXIT_FAILURE);
    }

    // Initialise ROS2
    rclcpp::init(argc, argv);  // Initialise ROS2
    std::cout << "Press enter to display shape.";
    std::cin.ignore();

    auto ros_worker = rclcpp::executors::MultiThreadedExecutor{};

    auto const config_strings = assignment3::ConfigReader{config_file};
    auto const config = assignment3::ConfigParser{config_strings};

    std::string zid = config.get_zid();

    int n_shapes = 0;

    // Initialise Landscape Shapes
    auto landscape_list_ptr =
        std::make_shared<visualization_msgs::msg::MarkerArray>();
    auto const landscape =
        std::make_shared<shapes::Landscape>(n_shapes++, landscape_list_ptr);
    auto landscape_display = std::make_shared<display::MultipleShapeDisplay>(
        "landscape", 5s, landscape_list_ptr);

    // Display Landscape
    landscape_display->display_object(landscape);
    // Add display node to list of nodes ros automatically manages.
    ros_worker.add_node(landscape_display);

    // Keep track of Unique ID.
    n_shapes += 20;

    // Initialise UAV Shapes.
    auto uav_list_ptr =
        std::make_shared<visualization_msgs::msg::MarkerArray>();
    auto const uav = std::make_shared<shapes::Uav>(
        n_shapes++, zid, config.get_joystick_config(), 100ms, uav_list_ptr);
    auto uav_display = std::make_shared<display::MultipleShapeDisplay>(
        "uav", 100ms, uav_list_ptr);

    // Display UAV
    uav_display->display_object(uav);

    // Add display node to list of nodes ros automatically manages.
    ros_worker.add_node(uav_display);
    ros_worker.add_node(uav);

    // Keep track of Unique ID.
    n_shapes += 30;

    int current_colour = 0;
    // Initialise cube the UAV is carrying.
    auto const carry_cube = shapes::cube_gen(n_shapes++, 1);
    auto carry_cube_display =
        std::make_shared<display::SingleShapeDisplay>("carry_cube", 100ms);

    // Iterate through a list of colours to cycle through all 6 of them.
    shapes::ColourInterface::Colour cc =
        static_cast<shapes::ColourInterface::Colour>(current_colour % 6);
    carry_cube->set_colour(cc);
    carry_cube_display->display_object(carry_cube);
    ros_worker.add_node(carry_cube_display);

    // Initialise cube_list of dropped cubes.
    auto cube_list_ptr =
        std::make_shared<visualization_msgs::msg::MarkerArray>();
    auto const cube_list = std::make_shared<shapes::CubeListMA>(
        zid, config.get_joystick_config(), 100ms, cube_list_ptr);
    auto cube_list_display = std::make_shared<display::MultipleShapeDisplay>(
        "cube_list", 100ms, cube_list_ptr);
    // Set link uav to the display output.
    uav_display->display_object(cube_list);
    // Add display node to list of nodes ros automatically manages.
    ros_worker.add_node(cube_list_display);
    ros_worker.add_node(cube_list);

    // Dropped Block Handling
    int block_dropped = 0;

    // Periodically do work checks to update scenery etc.
    while (rclcpp::ok()) {
      // Update Carry Cube.
      auto location = uav->get_location();
      carry_cube->move_to(
          (std::get<0>(location)), (std::get<1>(location)),
          shapes::ZAxis{(std::get<2>(location)).get_value() + 98});
      auto uav_one = (uav->get_display_list())->markers[0];
      auto& carry_cube_ptr = (*(carry_cube->get_display_markers()))[0];
      carry_cube_ptr.pose.orientation = uav_one.pose.orientation;

      // Check if drop button has been pressed since the last time it was
      // unpressed.
      block_dropped += cube_list->get_block_dropped();
      block_dropped *= cube_list->get_block_dropped();
      if (block_dropped >= 2) {
        block_dropped = 2;
      }

      // Check for colour changing block condition.
      if (block_dropped == 1) {
        // Drop block at carry_cube's location.
        cube_list->add_cube(n_shapes++, carry_cube->get_display_markers());

        // Pre-increment current_colour then assign it to the carried block.
        cc = static_cast<shapes::ColourInterface::Colour>(++current_colour % 6);
        carry_cube->set_colour(cc);
      }

      ros_worker.spin_some(200ms);
    }

  }

  catch (std::exception& e) {
    // Something wrong occured, printing error message.
    std::cerr << "Error message:" << e.what() << "\n";
  }

  rclcpp::shutdown();  // Cleaning up before exiting.

  return 0;
}

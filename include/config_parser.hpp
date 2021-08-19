// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#ifndef CONFIG_PARSER_HPP_
#define CONFIG_PARSER_HPP_

#include <chrono>
#include <istream>
#include <string>
#include <unordered_map>

namespace assignment3 {
struct JoystickConfig {
 public:
  std::size_t x_axis;
  std::size_t y_axis;
  std::size_t z_plus_axis;
  std::size_t z_minus_axis;
  std::size_t steering_axis;
  std::size_t drop_block_button;
  std::size_t clear_blocks_button;
  std::size_t gravity_button;
  double trigger_deadzone;
  double joystick_deadzone;
  double floor_bound;
  double ceiling_bound;
  double side_bound;
};

class ConfigReader {
 public:
  explicit ConfigReader(std::istream& config_file);
  [[nodiscard]] auto find_config(std::string const& key,
                                 std::string const& default_value) const
      -> std::string;

 private:
  std::unordered_map<std::string, std::string> config_;
};

class ConfigParser {
 public:
  explicit ConfigParser(ConfigReader const& config);

  [[nodiscard]] auto get_zid() const -> std::string;
  [[nodiscard]] auto get_refresh_period() const -> std::chrono::milliseconds;
  [[nodiscard]] auto get_joystick_config() const -> JoystickConfig;

 private:
  std::string const zid_;
  std::chrono::milliseconds const refresh_period_;
  JoystickConfig const joy_config_;
};
}  // namespace assignment3
#endif  // CONFIG_PARSER_HPP_

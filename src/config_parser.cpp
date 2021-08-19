// Copyright 2019 Zhihao Zhang License MIT
// Adapted for MTRN2500 Assignment 3 2019
// Dylan Sanusi-Goh and Joshua Behar

#include "config_parser.hpp"

#include <algorithm>  // std::remove_if
#include <iostream>
#include <sstream>  // std::istringstream
#include <string>
#include <unordered_map>

namespace assignment3 {
ConfigReader::ConfigReader(std::istream& config_file) {
  // Get line by line of config_file
  std::string key;
  std::string value;

  int n_lines = 0;
  while (std::getline(config_file, key, ':') &&
         std::getline(config_file, value)) {
    std::cout << "Line " << ++n_lines << ": " << key << ":" << value
              << std::endl;
    // Purge whitespace from lines

    key.erase(remove_if(key.begin(), key.end(), isspace), key.end());
    value.erase(remove_if(value.begin(), value.end(), isspace), value.end());

    // Add to unordered map config_
    config_[key] = value;
  }

  std::cout << std::endl;

  // Print out unordered map config_
  for (const auto& [key, value] : config_)
    std::cout << "key: \"" << key << "\", "
              << "value: \"" << value << "\"" << std::endl;

  std::cout << std::endl;
}

auto ConfigReader::find_config(std::string const& key,
                               std::string const& default_value) const
    -> std::string {
  auto config_iterator = config_.find(key);
  if (!(config_iterator == config_.end())) {
    return config_iterator->second;
  }

  std::cout << "Configuration for " + key +
                   " not found. Default configuration used."
            << std::endl;
  return default_value;
}

ConfigParser::ConfigParser(ConfigReader const& config)
    : zid_{config.find_config("zid", std::string{"z0000000"})},
      refresh_period_{std::chrono::duration<int64_t>(
          stol(config.find_config("refresh_rate", std::string{"10"})))},
      joy_config_{
          stoul(config.find_config("x_axis", std::string{"0"})),
          stoul(config.find_config("y_axis", std::string{"1"})),
          stoul(config.find_config("z_plus_axis", std::string{"2"})),
          stoul(config.find_config("z_minus_axis", std::string{"5"})),
          stoul(config.find_config("steering_axis", std::string{"3"})),
          stoul(config.find_config("drop_block_button", std::string{"2"})),
          stoul(config.find_config("clear_blocks_button", std::string{"5"})),
          stoul(config.find_config("gravity_button", std::string{"3"})),
          stod(config.find_config("trigger_deadzone", std::string{"0.1"})),
          stod(config.find_config("joystick_deadzone", std::string{"0.5"})),
          stod(config.find_config("floor_bound", std::string{"5"})),
          stod(config.find_config("ceiling_bound", std::string{"80"})),
          stod(config.find_config("side_bound", std::string{"25"}))} {}

auto ConfigParser::get_zid() const -> std::string { return zid_; }

auto ConfigParser::get_refresh_period() const -> std::chrono::milliseconds {
  return refresh_period_;
}

auto ConfigParser::get_joystick_config() const -> JoystickConfig {
  return joy_config_;
}

}  // namespace assignment3

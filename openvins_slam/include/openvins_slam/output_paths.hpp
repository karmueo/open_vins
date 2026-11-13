#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace openvins_slam {
struct OutputPaths {
  std::string trajectory;
  std::string mcap;
};

// Names use the original YAML basename, never a launch-generated parameter file.
inline OutputPaths output_paths(const std::string &directory, const std::string &input,
                                const std::string &config_name, bool save_mcap) {
  namespace fs = std::filesystem;
  if (directory.empty()) throw std::runtime_error("parameter outdir is required");
  if (input.empty() || fs::path(input).stem().empty()) throw std::runtime_error("parameter in_bag is required");
  if (config_name.find('/') != std::string::npos || config_name.find('\\') != std::string::npos)
    throw std::runtime_error("config_name must be a basename without directory separators");
  const auto prefix = "openvins_" + fs::path(input).stem().string() + "_" + config_name;
  return {(fs::path(directory) / (prefix + "_trajectory.txt")).string(),
          save_mcap ? (fs::path(directory) / (prefix + ".mcap")).string() : ""};
}
} // namespace openvins_slam

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "core/VioManagerOptions.h"

namespace openvins_slam {
// Load overrides and validate before creating the estimator or output files.
void configure_options(rclcpp::Node &node, ov_msckf::VioManagerOptions &options);
} // namespace openvins_slam

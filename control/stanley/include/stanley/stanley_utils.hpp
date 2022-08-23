// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the stanley_node class.

#ifndef STANLEY__STANLEY_UTILS_HPP_
#define STANLEY__STANLEY_UTILS_HPP_

#include "motion_utils/resample/resample.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using autoware_auto_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;

namespace autoware
{
namespace stanley
{
namespace utils
{
double normalizeEulerAngle(const double euler);

double calcHeading(const Pose & pose1, const Pose & pose2);

std::pair<size_t, double> calcClosestPoint(std::vector<Pose> & trajectory, Pose & pose);

double euclideanDistance(const Pose & pose1, const Pose & pose2);

geometry_msgs::msg::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  rclcpp::Logger & logger);

std::vector<Pose> extractPoses(const Trajectory & trajectory);

double calcYawRate(double velocity, double yaw, double wheelbase);

std::vector<Pose> createVirtualPath(std::vector<Pose> & path, double wheelbase, double interval);

}  // namespace utils
}  // namespace stanley
}  // namespace autoware
#endif  // STANLEY__STANLEY_UTILS_HPP_

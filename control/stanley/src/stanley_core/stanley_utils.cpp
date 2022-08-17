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

#include "stanley/stanley_utils.hpp"

namespace autoware
{
namespace stanley
{
namespace utils
{
double normalizeEulerAngle(const double euler)
{
  double normalized = euler;
  while (normalized > M_PI) {
    normalized -= 2.0 * M_PI;
  }
  while (normalized < -M_PI) {
    normalized += 2.0 * M_PI;
  }
  return normalized;
}

double calcCurvature(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
  return atan2(pose1.position.y - pose2.position.y, pose1.position.x - pose2.position.x);
}

double eucladianDistance(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
  return sqrt(
    pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
}

std::pair<size_t, double> calcClosestPoint(
  std::vector<geometry_msgs::msg::Pose> & trajectory, geometry_msgs::msg::Pose & pose)
{
  if (trajectory.size() < 2) {
    return std::make_pair(0, 0.0);
  }
  size_t closest_point_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    double distance = eucladianDistance(trajectory.at(i), pose);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point_index = i;
    }
  }
  return std::make_pair(closest_point_index, min_distance);
}

geometry_msgs::msg::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  rclcpp::Logger & logger)
{
  while (rclcpp::ok()) {
    try {
      const auto transform = tf_buffer.lookupTransform(from, to, tf2::TimePointZero);
      return transform;
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(logger, "waiting for transform from `%s` to `%s` ...", from.c_str(), to.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(5000));
    }
  }
  return geometry_msgs::msg::TransformStamped();
}

std::vector<geometry_msgs::msg::Pose> extractPoses(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  for (const auto & p : trajectory.points) {
    poses.push_back(p.pose);
  }

  return poses;
}

}  // namespace utils
}  // namespace stanley
}  // namespace autoware

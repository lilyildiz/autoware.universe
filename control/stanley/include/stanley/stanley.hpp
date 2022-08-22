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
/// \brief This file defines the stanley class.

#ifndef STANLEY__STANLEY_HPP_
#define STANLEY__STANLEY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stanley/stanley_utils.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware
{
namespace stanley
{

class Stanley
{
public:
  Stanley() : m_k(0.0), m_dist_to_fr_ax(0.0) {}
  ~Stanley() = default;

  rclcpp::Logger logger = rclcpp::get_logger("stanley");

  // Input setters
  void setTrajectory(const std::vector<geometry_msgs::msg::Pose> & trajectory);
  void setPose(const geometry_msgs::msg::Pose & pose);
  void setOdom(const nav_msgs::msg::Odometry & odom);
  void setK(const double k) { this->m_k = k; }
  void setDistToFrAx(const double dist) { this->m_dist_to_fr_ax = dist; }

  bool isReady() const;
  std::pair<bool, double> run();

private:
  // k The gain of the stanley controller.
  double m_k;
  double m_dist_to_fr_ax;

  std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> m_trajectory_ptr;
  std::shared_ptr<geometry_msgs::msg::Pose> m_pose_ptr;
  std::shared_ptr<nav_msgs::msg::Odometry> m_odom_ptr;
};

}  // namespace stanley
}  // namespace autoware

#endif  // STANLEY__STANLEY_HPP_

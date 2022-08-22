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
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <memory>
#include <utility>
#include <vector>

using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;

namespace autoware
{
namespace stanley
{

class Stanley
{
public:
  Stanley()
  : m_k(0.0),
    m_k_soft(0.0),
    m_k_d_yaw(0.0),
    m_k_d_steer(0.0),
    m_wheelbase_m(0.0),
    m_curr_steer(0.0),
    m_prev_steer(0.0)
  {
  }
  ~Stanley() = default;

  rclcpp::Logger logger = rclcpp::get_logger("stanley");

  // Input setters
  void setTrajectory(const std::vector<Pose> & trajectory);
  void setPose(const Pose & pose);
  void setOdom(const Odometry & odom);
  void setK(const double k) { this->m_k = k; }
  void setDistToFrAx(const double dist) { this->m_wheelbase_m = dist; }
  void setKSoft(const double k) { this->m_k_soft = k; }
  void setCurrentSteering(const double current_steering) { this->m_curr_steer = current_steering; }

  bool isReady() const;
  std::pair<bool, double> run();

private:
  // k The gain of the stanley controller.
  double m_k;
  double m_k_soft;
  double m_k_d_yaw;
  double m_k_d_steer;
  double m_wheelbase_m;
  double m_curr_steer;
  double m_prev_steer;

  std::shared_ptr<std::vector<Pose>> m_trajectory_ptr;
  std::shared_ptr<Pose> m_pose_ptr;
  std::shared_ptr<Odometry> m_odom_ptr;
};

}  // namespace stanley
}  // namespace autoware

#endif  // STANLEY__STANLEY_HPP_

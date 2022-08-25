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
using tier4_autoware_utils::calcCurvature;
using tier4_autoware_utils::getPoint;

namespace autoware
{
namespace stanley
{

struct Params
{
  double k_straight;
  double k_turn;
  double k_soft;
  double k_d_yaw;
  double k_d_steer;
  double wheelbase_m;
  double curvature_threshold;
  double curvature_calc_dist;
  double convergence_threshold;
  double max_steer_rad;
};

class Stanley
{
public:
  Stanley() : m_curr_steer(0.0), m_prev_steer(0.0) {}
  ~Stanley() = default;

  rclcpp::Logger logger = rclcpp::get_logger("stanley");

  // Input setters
  void setTrajectory(const std::vector<Pose> & trajectory);
  void setPose(const Pose & pose);
  void setOdom(const Odometry & odom);
  void setParams(const Params & params) { m_params = params; }
  void setCurrentSteering(const double current_steering) { this->m_curr_steer = current_steering; }

  bool isReady() const;
  std::pair<bool, double> run();

private:
  // Parameters
  Params m_params;

  // State
  double m_curr_steer;
  double m_prev_steer;

  // Inputs
  std::shared_ptr<std::vector<Pose>> m_trajectory_ptr;
  std::shared_ptr<Pose> m_pose_ptr;
  std::shared_ptr<Odometry> m_odom_ptr;
};

}  // namespace stanley
}  // namespace autoware

#endif  // STANLEY__STANLEY_HPP_

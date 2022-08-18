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

#include "stanley/stanley.hpp"

#include <iostream>

namespace autoware
{
namespace stanley
{

void Stanley::setTrajectory(const std::vector<geometry_msgs::msg::Pose> & trajectory)
{
  m_trajectory_ptr = std::make_shared<std::vector<geometry_msgs::msg::Pose>>();
  *m_trajectory_ptr = trajectory;
}

void Stanley::setPose(const geometry_msgs::msg::Pose & pose)
{
  m_pose_ptr = std::make_shared<geometry_msgs::msg::Pose>();
  *m_pose_ptr = pose;
}

void Stanley::setOdom(const nav_msgs::msg::Odometry & odom)
{
  m_odom_ptr = std::make_shared<nav_msgs::msg::Odometry>();
  *m_odom_ptr = odom;
}

bool Stanley::isReady() const
{
  return m_trajectory_ptr != nullptr && m_pose_ptr != nullptr && m_odom_ptr != nullptr;
}


std::pair<bool, double> Stanley::run()
{
  // Check if we have enough data to run the algorithm
  if (!isReady()) {
    RCLCPP_ERROR(logger, "Inputs are not ready");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }
  if (m_trajectory_ptr->size() < 2) {
    RCLCPP_ERROR(logger, "Trajectory is too short");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  // Get heading error
  double vehicle_yaw = tf2::getYaw(m_pose_ptr->orientation);
  double trajectory_yaw = utils::calcCurvature(m_trajectory_ptr->back(), m_trajectory_ptr->at(0));
  double trajectory_yaw_error = utils::normalizeEulerAngle(vehicle_yaw - trajectory_yaw);

  // Get the closest point
  std::pair<size_t, double> closest_point = utils::calcClosestPoint(*m_trajectory_ptr, *m_pose_ptr);

  // Calculate cross track error
  double cross_track_yaw = utils::calcCurvature(*m_pose_ptr, m_trajectory_ptr->at(closest_point.first));
  double cross_track_yaw_diff = utils::normalizeEulerAngle(trajectory_yaw - cross_track_yaw);
  double cross_track_error =
    (cross_track_yaw_diff > 0) ? abs(closest_point.second) : -abs(closest_point.second);

  double cross_track_yaw_error = atan(m_k * cross_track_error / m_odom_ptr->twist.twist.linear.x);

  // Calculate the steering angle
  double steering_angle = utils::normalizeEulerAngle(cross_track_yaw_error + trajectory_yaw_error);

  return std::make_pair(true, steering_angle);
}

}  // namespace stanley
}  // namespace autoware

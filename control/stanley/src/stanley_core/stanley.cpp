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

namespace autoware
{
namespace stanley
{

void Stanley::setTrajectory(const std::vector<Pose> & trajectory)
{
  m_trajectory_ptr = std::make_shared<std::vector<Pose>>();
  *m_trajectory_ptr = trajectory;
}

void Stanley::setPose(const Pose & pose)
{
  m_pose_ptr = std::make_shared<Pose>();
  *m_pose_ptr = pose;
}

void Stanley::setOdom(const Odometry & odom)
{
  m_odom_ptr = std::make_shared<Odometry>();
  *m_odom_ptr = odom;
}

bool Stanley::isReady() const
{
  return m_trajectory_ptr != nullptr && m_pose_ptr != nullptr && m_odom_ptr != nullptr &&
         m_wheelbase_m != 0.0;
}

std::pair<bool, double> Stanley::run()
{
  // Check if we have enough data to run the algorithm
  if (!isReady()) {
    RCLCPP_ERROR(logger, "[Stanley]Inputs are not ready");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }
  // temp
  m_k_soft = 1.00;
  m_k_d_yaw = 0.09;
  m_k_d_steer = 0.45;

  // Get front axle pose
  Pose front_axle_pose = tier4_autoware_utils::calcOffsetPose(*m_pose_ptr, m_wheelbase_m, 0.0, 0.0);

  // Get the closest point to front axle
  std::pair<size_t, double> closest_point =
    utils::calcClosestPoint(*m_trajectory_ptr, front_axle_pose);

  // Check if we have enough points to run the algorithm
  std::vector<Pose> cropped_vec = {
    m_trajectory_ptr->begin() + closest_point.first, m_trajectory_ptr->end()};
  RCLCPP_ERROR(logger, "PAth size: %ld", cropped_vec.size());
  if (cropped_vec.size() < 2) {
    RCLCPP_ERROR(logger, "[Stanley]Trajectory is too short");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  // Get heading error
  double vehicle_yaw = tf2::getYaw(front_axle_pose.orientation);
  double trajectory_yaw = tf2::getYaw(m_trajectory_ptr->at(closest_point.first).orientation);
  double trajectory_yaw_error = utils::normalizeEulerAngle(trajectory_yaw - vehicle_yaw);

  // Calculate cross track error
  double cross_track_yaw =
    utils::calcHeading(front_axle_pose, m_trajectory_ptr->at(closest_point.first));
  double cross_track_yaw_diff = utils::normalizeEulerAngle(trajectory_yaw - cross_track_yaw);
  double cross_track_error =
    (cross_track_yaw_diff > 0) ? abs(closest_point.second) : -abs(closest_point.second);

  double cross_track_yaw_error =
    atan(m_k * cross_track_error / (m_odom_ptr->twist.twist.linear.x + m_k_soft));

  // Negative feedback on yaw rate
  double measured_yaw_rate =
    utils::calcYawRate(m_odom_ptr->twist.twist.linear.x, vehicle_yaw, m_wheelbase_m);
  double trajectory_yaw_rate =
    utils::calcYawRate(m_odom_ptr->twist.twist.linear.x, trajectory_yaw, m_wheelbase_m);
  double yaw_feedback = -m_k_d_yaw * (measured_yaw_rate - trajectory_yaw_rate);

  // Steer damping
  double steer_damp = m_k_d_steer * (m_prev_steer - m_curr_steer);

  // Calculate the steering angle
  double steering_angle = utils::normalizeEulerAngle(
    cross_track_yaw_error + trajectory_yaw_error + yaw_feedback + steer_damp);

  m_prev_steer = m_curr_steer;

  RCLCPP_ERROR(logger, "Cross track yaw error: %f", cross_track_yaw_error);
  RCLCPP_ERROR(logger, "Trajectory yaw error: %f", trajectory_yaw_error);
  RCLCPP_ERROR(logger, "Yaw feedback: %f", yaw_feedback);
  RCLCPP_ERROR(logger, "Steer damping: %f", steer_damp);
  RCLCPP_ERROR(logger, "Steering angle: %f", steering_angle);
  RCLCPP_ERROR(logger, "m_k: %f", m_k);

  return std::make_pair(true, steering_angle);
}

}  // namespace stanley
}  // namespace autoware

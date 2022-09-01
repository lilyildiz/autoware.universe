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

#include <math.h>

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
         m_params.wheelbase_m != 0.0;
}

std::pair<bool, double> Stanley::run()
{
  // Check if we have enough data to run the algorithm
  if (!isReady()) {
    RCLCPP_ERROR(logger, "[Stanley]Inputs are not ready");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  // Get driving direction
  const auto is_forward_shift =
    tier4_autoware_utils::isDrivingForward(m_trajectory_ptr->at(0), m_trajectory_ptr->at(1));

  // Append virtual points to trajectory
  if (is_forward_shift) {
    const auto virtual_path =
      utils::createVirtualPath(*m_trajectory_ptr, m_params.wheelbase_m, 0.5);
    m_trajectory_ptr->insert(m_trajectory_ptr->end(), virtual_path.begin(), virtual_path.end());
  }

  // Path smoothing
  if (m_params.enable_path_smoothing) {
    const auto smoothed_path =
      utils::smoothPath(*m_trajectory_ptr, m_params.path_filter_moving_ave_num, is_forward_shift);
    m_trajectory_ptr->assign(smoothed_path.begin(), smoothed_path.end());
  }

  // Get front axle pose
  Pose front_axle_pose =
    tier4_autoware_utils::calcOffsetPose(*m_pose_ptr, m_params.wheelbase_m, 0.0, 0.0);

  // Pose for steering calculation
  Pose steering_pose = is_forward_shift ? front_axle_pose : *m_pose_ptr;

  // Get the closest point to front axle
  std::pair<size_t, double> closest_point =
    utils::calcClosestPoint(*m_trajectory_ptr, steering_pose);

  // Calculate trajectory curvature
  double curvature = 0.0;
  size_t p2_index =
    utils::getNextIdxWithThr(*m_trajectory_ptr, closest_point.first, m_params.curvature_calc_dist);
  size_t p3_index =
    utils::getNextIdxWithThr(*m_trajectory_ptr, p2_index, m_params.curvature_calc_dist);
  if (
    p2_index > m_trajectory_ptr->size() - 1 || p3_index > m_trajectory_ptr->size() - 1 ||
    p2_index == p3_index) {
    RCLCPP_ERROR(logger, "[Stanley]Trajectory is too short for curvature calculation");
    curvature = std::numeric_limits<double>::quiet_NaN();
  } else {
    curvature = calcCurvature(
      getPoint(m_trajectory_ptr->at(closest_point.first)), getPoint(m_trajectory_ptr->at(p2_index)),
      getPoint(m_trajectory_ptr->at(p3_index)));
  }

  // Get heading error
  double vehicle_yaw = tf2::getYaw(steering_pose.orientation);
  double trajectory_yaw = tf2::getYaw(m_trajectory_ptr->at(closest_point.first).orientation);
  double trajectory_yaw_error = utils::normalizeEulerAngle(trajectory_yaw - vehicle_yaw);

  // Calculate cross track error
  double cross_track_yaw =
    utils::calcHeading(steering_pose, m_trajectory_ptr->at(closest_point.first));
  double cross_track_yaw_diff = utils::normalizeEulerAngle(trajectory_yaw - cross_track_yaw);
  double cross_track_error =
    (cross_track_yaw_diff > 0) ? abs(closest_point.second) : -abs(closest_point.second);

  // Set gains according to curvature and gear
  double k =
    is_forward_shift
      ? (fabs(curvature) > m_params.curvature_threshold ? m_params.k_turn : m_params.k_straight)
      : m_params.reverse_k;
  double k_soft = is_forward_shift ? m_params.k_soft : m_params.reverse_k_soft;
  double k_d_yaw = is_forward_shift ? m_params.k_d_yaw : m_params.reverse_k_d_yaw;

  // Calculate cross track yaw error
  double cross_track_yaw_error =
    atan(k * cross_track_error / (m_odom_ptr->twist.twist.linear.x + k_soft));

  // Negative feedback on yaw rate
  double measured_yaw_rate =
    utils::calcYawRate(m_odom_ptr->twist.twist.linear.x, vehicle_yaw, m_params.wheelbase_m);
  double trajectory_yaw_rate =
    utils::calcYawRate(m_odom_ptr->twist.twist.linear.x, trajectory_yaw, m_params.wheelbase_m);
  double yaw_feedback = k_d_yaw * (measured_yaw_rate - trajectory_yaw_rate);

  // Steer damping
  double steer_damp = m_params.k_d_steer * (m_prev_steer - m_curr_steer);

  // Calculate the steering angle
  double steering_angle = utils::normalizeEulerAngle(
    cross_track_yaw_error + trajectory_yaw_error + yaw_feedback + steer_damp);

  // Reverse steering angle if driving in reverse gear
  steering_angle = is_forward_shift ? steering_angle : -steering_angle;

  m_prev_steer = m_curr_steer;

  RCLCPP_ERROR(logger, "Cross track yaw error: %f", cross_track_yaw_error);
  RCLCPP_ERROR(logger, "Trajectory yaw error: %f", trajectory_yaw_error);
  RCLCPP_ERROR(logger, "Yaw feedback: %f", yaw_feedback);
  RCLCPP_ERROR(logger, "Steer damping: %f", steer_damp);
  RCLCPP_ERROR(logger, "Steering angle: %f", steering_angle);
  RCLCPP_ERROR(logger, "k: %f", k);
  RCLCPP_ERROR(logger, "k_soft: %f", k_soft);
  RCLCPP_ERROR(logger, "k_d_yaw: %f", k_d_yaw);
  RCLCPP_ERROR(logger, "k_d_steer: %f", m_params.k_d_steer);
  RCLCPP_ERROR(logger, "Path curvature: %f", curvature);
  RCLCPP_ERROR(logger, "Enable path smoothing: %d", m_params.enable_path_smoothing);
  RCLCPP_ERROR(logger, "Path filter moving average: %ld", m_params.path_filter_moving_ave_num);
  RCLCPP_ERROR(logger, "Is Forward: %d", is_forward_shift);
  RCLCPP_ERROR(logger, "Vehicle yaw: %f", vehicle_yaw);
  RCLCPP_ERROR(logger, "Trajectory yaw: %f", trajectory_yaw);

  return std::make_pair(true, steering_angle);
}

}  // namespace stanley
}  // namespace autoware

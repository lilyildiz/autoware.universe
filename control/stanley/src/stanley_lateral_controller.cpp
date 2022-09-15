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

#include "stanley/stanley_lateral_controller.hpp"

namespace autoware
{
namespace stanley
{

StanleyLateralController::StanleyLateralController(rclcpp::Node & node)
: m_node(&node), m_tf_buffer(node.get_clock()), m_tf_listener(m_tf_buffer), m_pose_listener(&node)
{
  // Algorithm object
  m_stanley = std::make_unique<Stanley>();

  // Parameters
  m_params.k_straight = m_node->declare_parameter("k_straight", 0.0);
  m_params.k_turn = m_node->declare_parameter<double>("k_turn", 0.0);
  m_params.k_soft = m_node->declare_parameter<double>("k_soft", 0.0);
  m_params.k_d_yaw = m_node->declare_parameter<double>("k_d_yaw", 0.0);
  m_params.k_d_steer = m_node->declare_parameter<double>("k_d_steer", 0.0);
  m_params.reverse_k = m_node->declare_parameter<double>("reverse_k", 0.0);
  m_params.reverse_k_soft = m_node->declare_parameter<double>("reverse_k_soft", 0.0);
  m_params.reverse_k_d_yaw = m_node->declare_parameter<double>("reverse_k_d_yaw", 0.0);
  m_params.recover_k = m_node->declare_parameter<double>("recover_k", 0.0);
  m_params.curvature_threshold = m_node->declare_parameter<double>("curvature_threshold", 0.0);
  m_params.curvature_calc_index = m_node->declare_parameter<int64_t>("curvature_calc_index", 0.0);
  m_params.convergence_threshold = m_node->declare_parameter<double>("convergence_threshold", 0.1);
  m_params.max_steer_rad =
    vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo().max_steer_angle_rad;
  m_params.wheelbase_m = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo().wheel_base_m;
  m_params.wheel_tread_m = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo().wheel_tread_m;
  m_params.enable_path_smoothing = m_node->declare_parameter<bool>("enable_path_smoothing", false);
  m_params.path_filter_moving_ave_num =
    m_node->declare_parameter<int64_t>("path_filter_moving_ave_num", 5);

  // Wait for pose
  stanley_utils::waitForTransform(m_tf_buffer, "map", "base_link", logger);
}

StanleyLateralController::~StanleyLateralController() = default;

void StanleyLateralController::setInputData(const InputData & input_data)
{
  m_trajectory = input_data.current_trajectory_ptr;
  m_odom = input_data.current_odometry_ptr;
  m_steering_report = input_data.current_steering_ptr;
}

bool StanleyLateralController::isReady()
{
  if (!m_odom) {
    RCLCPP_WARN_THROTTLE(
      m_node->get_logger(), *m_node->get_clock(), 5000, "waiting for current_odometry...");
    return false;
  }

  if (!m_trajectory) {
    RCLCPP_WARN_THROTTLE(
      m_node->get_logger(), *m_node->get_clock(), 5000, "waiting for trajectory...");
    return false;
  }

  if (!m_pose) {
    RCLCPP_WARN_THROTTLE(
      m_node->get_logger(), *m_node->get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  return true;
}

boost::optional<LateralOutput> StanleyLateralController::run()
{
  // Get pose
  m_pose = m_pose_listener.getCurrentPose();

  // Check inputs
  if (!isReady()) {
    RCLCPP_ERROR(logger, "[Stanley Lateral Controller]Inputs are not ready");
    return boost::none;
  }

  // Set Stanley inputs

  m_stanley->setTrajectory(*m_trajectory);
  m_stanley->setOdom(*m_odom);
  m_stanley->setPose(m_pose->pose);
  m_stanley->setParams(m_params);
  m_stanley->setCurrentSteering(m_steering_report->steering_tire_angle);

  // Run Stanley
  std::pair<bool, double> stanley_result = m_stanley->run();
  if (!stanley_result.first) {
    RCLCPP_ERROR(logger, "Stanley failed");
    return boost::none;
  }

  // Generate control_command
  AckermannLateralCommand control_command;
  control_command.stamp = m_node->get_clock()->now();
  control_command.steering_tire_angle = static_cast<float>(
    stanley_utils::limitSteerAngle(stanley_result.second, m_params.max_steer_rad));

  // Generate output
  LateralOutput output;
  output.control_cmd = control_command;
  output.sync_data.is_steer_converged =
    std::abs(control_command.steering_tire_angle - m_steering_report->steering_tire_angle) <
    m_params.convergence_threshold;

  return output;
}

}  // namespace stanley
}  // namespace autoware

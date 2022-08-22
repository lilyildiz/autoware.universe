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
  m_k = m_node->declare_parameter<double>("k", 1.0);
  m_convergence_threshold = m_node->declare_parameter<double>("convergence_threshold", 0.1);

  // Wait for pose
  utils::waitForTransform(m_tf_buffer, "map", "base_link", logger);

  // Vehicle Parameters
  m_vehicle_info = vehicle_info_util::VehicleInfoUtil(*m_node).getVehicleInfo();
}

StanleyLateralController::~StanleyLateralController() {}

void StanleyLateralController::setInputData(const InputData & input_data)
{
  m_trajectory = input_data.current_trajectory_ptr;
  m_odom = input_data.current_odometry_ptr;
  m_steering_report = input_data.current_steering_ptr;
}

bool StanleyLateralController::isReady()
{
  return m_trajectory != nullptr && m_odom != nullptr && m_steering_report != nullptr;
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
  m_stanley->setK(m_k);
  m_stanley->setTrajectory(utils::extractPoses(*m_trajectory));
  m_stanley->setOdom(*m_odom);
  m_stanley->setPose(m_pose->pose);
  m_stanley->setDistToFrAx(m_vehicle_info.wheel_base_m);
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
  control_command.steering_tire_angle = stanley_result.second;

  // Generate output
  LateralOutput output;
  output.control_cmd = control_command;
  output.sync_data.is_steer_converged =
    std::abs(control_command.steering_tire_angle - m_steering_report->steering_tire_angle) <
    m_convergence_threshold;

  return output;
}

}  // namespace stanley
}  // namespace autoware

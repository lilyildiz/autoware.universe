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

#ifndef STANLEY__STANLEY_LATERAL_CONTROLLER_HPP_
#define STANLEY__STANLEY_LATERAL_CONTROLLER_HPP_

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tier4_autoware_utils/ros/self_pose_listener.hpp"
#include "trajectory_follower/lateral_controller_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stanley/stanley.hpp>
#include <stanley/stanley_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerBase;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

namespace autoware
{
namespace stanley
{

class StanleyLateralController : public LateralControllerBase
{
public:
  /**
   * @brief constructor
   */
  explicit StanleyLateralController(rclcpp::Node & node);

  /**
   * @brief destructor
   */
  virtual ~StanleyLateralController();

private:
  // Node
  rclcpp::Node::SharedPtr m_node;
  rclcpp::Logger logger = rclcpp::get_logger("stanley");
  /**
   * @brief check input data
   */
  bool isReady();

  // Input data
  Trajectory::ConstSharedPtr m_trajectory;
  Odometry::ConstSharedPtr m_odom;
  SteeringReport::ConstSharedPtr m_steering_report;
  /**
   * @brief set input data
   */
  void setInputData(InputData const & input_data) override;

  // TF
  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  tier4_autoware_utils::SelfPoseListener m_pose_listener;
  PoseStamped::ConstSharedPtr m_pose;

  // Algorithm
  std::unique_ptr<Stanley> m_stanley;

  // Parameters
  double m_k;
  double m_convergence_threshold;
  vehicle_info_util::VehicleInfo m_vehicle_info;
  double m_max_steer_rad;

  // Compute
  boost::optional<LateralOutput> run() override;
};
}  // namespace stanley
}  // namespace autoware

#endif  // STANLEY__STANLEY_LATERAL_CONTROLLER_HPP_

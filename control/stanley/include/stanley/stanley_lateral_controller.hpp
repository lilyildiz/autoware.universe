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

  /**
   * @brief check input data
   */
  bool isReady();

  /**
   * @brief set input data
   */
  void setInputData(InputData const & input_data) override;

  /**
   * @brief computing function
   */
  boost::optional<LateralOutput> run() override;

private:
  /* ROS */
  //!< @brief ROS node handle
  rclcpp::Node::SharedPtr m_node;
  //!< @brief ROS logger used for debug logging
  rclcpp::Logger logger = rclcpp::get_logger("stanley");

  /* Inputs */
  //!< @brief current trajectory
  Trajectory::ConstSharedPtr m_trajectory;
  //!< @brief current odometry
  Odometry::ConstSharedPtr m_odom;
  //!< @brief current steering report
  SteeringReport::ConstSharedPtr m_steering_report;

  /* TF */
  //!< @brief TF buffer
  tf2_ros::Buffer m_tf_buffer;
  //!< @brief TF listener
  tf2_ros::TransformListener m_tf_listener;
  //!< @brief self pose listener
  tier4_autoware_utils::SelfPoseListener m_pose_listener;
  //!< @brief current vehicle pose
  PoseStamped::ConstSharedPtr m_pose;

  /* Algorithm */
  //!< @brief Stanley algorithm object
  std::unique_ptr<Stanley> m_stanley;

  /* Parameters */
  //!< @brief stanley parameters
  Params m_params{};

};
}  // namespace stanley
}  // namespace autoware

#endif  // STANLEY__STANLEY_LATERAL_CONTROLLER_HPP_

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
  //!< @brief stanley cross track gain for straights
  double k_straight;
  //!< @brief stanley cross track gain for turns
  double k_turn;
  //!< @brief stanley gain for low speed
  double k_soft;
  //!< @brief stanley gain for negative yaw rate feedback
  double k_d_yaw;
  //!< @brief stanley gain for steer damping
  double k_d_steer;
  //!< @brief vehicle wheelbase[m]
  double wheelbase_m;
  //!< @brief curvature threshold for turns
  double curvature_threshold;
  //!< @brief point-to-point distance used in curvature calculation
  double curvature_calc_dist;
  //!< @brief convergence threshold for stanley controller[m]
  double convergence_threshold;
  //!< @brief maximum steering angle[rad]
  double max_steer_rad;
  //!< @brief path smoothing flag
  bool enable_path_smoothing;
  //!< @brief index distance of the moving average filter
  int64_t path_filter_moving_ave_num;
};

/**
 * Stanley waypoints follower class
 * @brief calculate steering angle to follow reference trajectory
 */
class Stanley
{
public:
  /**
   * @brief constructor
   */
  Stanley() = default;

  /**
   * @brief destructor
   */
  ~Stanley() = default;

  /* Input setters */
  /**
   * @brief set current trajectory
   * @param trajectory current trajectory
   */
  void setTrajectory(const std::vector<Pose> & trajectory);

  /**
   * @brief set current pose
   * @param pose current pose
   */
  void setPose(const Pose & pose);

  /**
   * @brief set current odometry
   * @param odom current odometry
   */
  void setOdom(const Odometry & odom);

  /**
   * @brief set stanley algorithm parameters
   * @param params stanley algorithm parameters
   */
  void setParams(const Params & params) { m_params = params; }

  /**
   * @brief set current steering angle
   * @param steer current steering angle
   */
  void setCurrentSteering(const double current_steering) { this->m_curr_steer = current_steering; }

  /* Compute */
  /**
   * @brief check input data
   * @return bool true if input data is ready
   */
  bool isReady() const;

  /**
   * @brief compute stanley steering angle
   * @return stanley steering angle
   */
  std::pair<bool, double> run();

private:
  /* Parameters */
  //!< @brief stanley algorithm parameters
  Params m_params{};

  /* State */
  //!< @brief current steering angle
  double m_curr_steer{0.0};
  //!< @brief previous steering angle
  double m_prev_steer{0.0};

  /* Inputs */
  //!< @brief current trajectory
  std::shared_ptr<std::vector<Pose>> m_trajectory_ptr;
  //!< @brief current pose
  std::shared_ptr<Pose> m_pose_ptr;
  //!< @brief current odometry
  std::shared_ptr<Odometry> m_odom_ptr;

  //!< @brief ROS logger used for debug logging
  rclcpp::Logger logger = rclcpp::get_logger("stanley");
};

}  // namespace stanley
}  // namespace autoware

#endif  // STANLEY__STANLEY_HPP_

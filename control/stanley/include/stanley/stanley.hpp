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

#include "motion_utils/trajectory/tmp_conversion.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stanley/stanley_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <memory>
#include <utility>
#include <vector>

using autoware_auto_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;
using motion_velocity_smoother::trajectory_utils::calcTrajectoryCurvatureFrom3Points;
using nav_msgs::msg::Odometry;

namespace autoware
{
namespace stanley
{

struct Params
{
  double k_straight;       //!< @brief stanley cross track gain for straights
  double k_turn;           //!< @brief stanley cross track gain for turns
  double k_soft;           //!< @brief stanley gain for low speed
  double k_d_yaw;          //!< @brief stanley gain for negative yaw rate feedback
  double k_d_steer;        //!< @brief stanley gain for steer damping
  double reverse_k;        //!< @brief stanley cross track gain for reverse gear
  double reverse_k_soft;   //!< @brief stanley gain for low speed in reverse gear
  double reverse_k_d_yaw;  //!< @brief stanley gain for negative yaw rate feedback for reverse gear
  double recover_k;        //!< @brief stanley cross track gain for recovery
  double wheelbase_m;      //!< @brief vehicle wheelbase[m]
  double wheel_tread_m;    //!< @brief distance between left wheel center and right wheel center[m]
  double curvature_threshold;          //!< @brief curvature threshold for turns
  int64_t curvature_calc_index;        //!< @brief index distance used in curvature calculation
  double convergence_threshold;        //!< @brief convergence threshold for stanley controller[m]
  double max_steer_rad;                //!< @brief maximum steering angle[rad]
  bool enable_path_smoothing;          //!< @brief path smoothing flag
  int64_t path_filter_moving_ave_num;  //!< @brief index distance of the moving average filter
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
  Stanley();

  /**
   * @brief destructor
   */
  ~Stanley() = default;

  /* Input setters */
  /**
   * @brief set current trajectory
   * @param trajectory current trajectory
   */
  void setTrajectory(const Trajectory & trajectory) { *m_trajectory_ptr = trajectory; };

  /**
   * @brief set current pose
   * @param pose current pose
   */
  void setPose(const Pose & pose) { *m_pose_ptr = pose; };

  /**
   * @brief set current odometry
   * @param odom current odometry
   */
  void setOdom(const Odometry & odom) { *m_odom_ptr = odom; };

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
  double m_curr_steer{};
  //!< @brief previous steering angle
  double m_prev_steer{};

  /* Inputs */
  //!< @brief current trajectory
  std::shared_ptr<Trajectory> m_trajectory_ptr;
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

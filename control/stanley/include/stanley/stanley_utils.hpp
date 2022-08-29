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
/// \brief This file defines the stanley_node class.

#ifndef STANLEY__STANLEY_UTILS_HPP_
#define STANLEY__STANLEY_UTILS_HPP_

#include "motion_utils/resample/resample.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/lowpass_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using autoware::motion::control::trajectory_follower::MoveAverageFilter::filt_vector;
using autoware_auto_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;

namespace autoware
{
namespace stanley
{
namespace utils
{
/**
 * @brief normalize the angle to [-pi, pi]
 * @param angle the angle to be normalized
 * @return the normalized angle
 */
double normalizeEulerAngle(const double euler);

/**
 * @brief calculate the heading angle from pose1 to pose2
 * @param pose1 first pose
 * @param pose2 second pose
 * @return heading angle from pose1 to pose2
 */
double calcHeading(const Pose & pose1, const Pose & pose2);

/**
 * @brief calculate the closest point on a trajectory to a given pose
 * @param trajectory the trajectory to be evaluated
 * @param pose the pose to be evaluated
 * @return index of the closest point on the trajectory to the given pose and the distance to the
 * closest point
 */
std::pair<size_t, double> calcClosestPoint(std::vector<Pose> & trajectory, Pose & pose);

/**
 * @brief calculate the euclidean distance between two poses
 * @param pose1 first pose
 * @param pose2 second pose
 * @return the euclidean distance between the two poses
 */
double euclideanDistance(const Pose & pose1, const Pose & pose2);

/**
 * @brief wait for a transform from one frame to another
 * @param tf_buffer the tf2 buffer
 * @param from the frame to transform from
 * @param to the frame to transform to
 * @param logger the logger to use
 * @return the transform
 */
geometry_msgs::msg::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  rclcpp::Logger & logger);

/**
 * @brief get Pose vector from a Trajectory
 * @param trajectory
 * @return Pose vector
 */
std::vector<Pose> extractPoses(const Trajectory & trajectory);

/**
 * @brief calculate yaw rate
 * @param velocity velocity
 * @param yaw yaw
 * @param wheelbase wheelbase
 * @return yaw rate
 */
double calcYawRate(double velocity, double yaw, double wheelbase);

/**
 * @brief create a virtual path with the length of the wheelbase at the end of the trajectory
 * @param path the path to be extended
 * @param wheelbase the wheelbase of the vehicle
 * @param interval waypoint interval
 * @return virtual path
 */
std::vector<Pose> createVirtualPath(std::vector<Pose> & path, double wheelbase, double interval);

/**
 * @brief limit steering angle
 * @param steer_angle the steering angle to be limited
 * @param max_angle the maximum steering angle
 * @return final steering angle
 */
double limitSteerAngle(double steer_angle, double max_angle);

/**
 * @brief get a point [threshold] meters ahead of the [starting_index] point on the [trajectory]
 * @param path the path to be evaluated
 * @param starting_index the index of the starting point
 * @param threshold the threshold distance
 * @return index of the calculated point
 */
size_t getNextIdxWithThr(std::vector<Pose> & path, size_t & starting_index, double threshold);

/**
 * @brief path smoothing
 * @param path the path to be smoothed
 * @param path_filter_moving_ave_num index distance of the moving average filter
 * @return smoothed path
 */
std::vector<Pose> smoothPath(std::vector<Pose> & path, int64_t path_filter_moving_ave_num);

}  // namespace utils
}  // namespace stanley
}  // namespace autoware
#endif  // STANLEY__STANLEY_UTILS_HPP_

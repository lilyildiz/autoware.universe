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

#include "stanley/stanley_utils.hpp"

namespace autoware
{
namespace stanley
{
namespace stanley_utils
{
double normalizeEulerAngle(const double euler)
{
  double normalized = euler;
  while (normalized > M_PI) {
    normalized -= 2.0 * M_PI;
  }
  while (normalized < -M_PI) {
    normalized += 2.0 * M_PI;
  }
  return normalized;
}

double calcHeading(const Pose & pose1, const Pose & pose2)
{
  return atan2(pose1.position.y - pose2.position.y, pose1.position.x - pose2.position.x);
}

double euclideanDistance(const Pose & pose1, const Pose & pose2)
{
  return sqrt(
    pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
}

std::pair<size_t, double> calcClosestPoint(std::vector<Pose> & trajectory, Pose & pose)
{
  if (trajectory.size() < 2) {
    return std::make_pair(0, 0.0);
  }
  size_t closest_point_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    double distance = euclideanDistance(trajectory.at(i), pose);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point_index = i;
    }
  }
  return std::make_pair(closest_point_index, min_distance);
}

geometry_msgs::msg::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  rclcpp::Logger & logger)
{
  while (rclcpp::ok()) {
    try {
      const auto transform = tf_buffer.lookupTransform(from, to, tf2::TimePointZero);
      return transform;
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(logger, "waiting for transform from `%s` to `%s` ...", from.c_str(), to.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(5000));
    }
  }
  return geometry_msgs::msg::TransformStamped();
}

std::vector<Pose> extractPoses(const Trajectory & trajectory)
{
  std::vector<Pose> poses;

  for (const auto & p : trajectory.points) {
    poses.push_back(p.pose);
  }

  return poses;
}

double calcYawRate(double velocity, double yaw, double wheelbase)
{
  return -(velocity * std::sin(yaw)) / wheelbase;
}

std::vector<Pose> createVirtualPath(std::vector<Pose> & path, double wheelbase, double interval)
{
  std::vector<Pose> virtual_path;
  virtual_path.push_back(path.back());
  virtual_path.push_back(
    tier4_autoware_utils::calcOffsetPose(path.back(), wheelbase + 2.0, 0.0, 0.0));
  std::vector<double> resample_archlenghts;
  for (double i = 0; i < wheelbase; i += interval) {
    resample_archlenghts.push_back(i);
  }
  return motion_utils::resamplePath(virtual_path, resample_archlenghts, true, false);
}

double limitSteerAngle(double steer_angle, double max_angle)
{
  if (steer_angle > max_angle) {
    return max_angle;
  }
  if (steer_angle < -max_angle) {
    return -max_angle;
  }
  return steer_angle;
}

double getPointCurvature(std::vector<Pose> & path, size_t idx_dist, size_t starting_index)
{
  if (path.size() < 3) {
    return 0.0;
  }

  // if the idx size is not enough, change the idx_dist
  const auto max_idx_dist = static_cast<size_t>(std::floor((path.size() - 1) / 2.0));
  idx_dist = std::max(1ul, std::min(idx_dist, max_idx_dist));

  if (idx_dist < 1) {
    throw std::logic_error("idx_dist less than 1 is not expected");
  }

  // calculate curvature by circle fitting from three points
  try {
    const auto p0 = getPoint(path.at(starting_index - idx_dist));
    const auto p1 = getPoint(path.at(starting_index));
    const auto p2 = getPoint(path.at(starting_index + idx_dist));
    return calcCurvature(p0, p1, p2);
  } catch (...) {
    return 0.0;  // points are too close. No curvature.
  }
}

std::vector<Pose> smoothPath(
  std::vector<Pose> & path, int64_t path_filter_moving_ave_num, bool is_forward)
{
  std::vector<Pose> smoothed_path = path;
  // Vectorize path
  std::vector<double> x_vector;
  std::vector<double> y_vector;
  for (auto & pose : path) {
    x_vector.push_back(pose.position.x);
    y_vector.push_back(pose.position.y);
  }

  // Path Smoothing
  if (path.size() > static_cast<size_t>(2 * path_filter_moving_ave_num)) {
    if (
      !filt_vector(path_filter_moving_ave_num, x_vector) ||
      !filt_vector(path_filter_moving_ave_num, y_vector)) {
      RCLCPP_ERROR(rclcpp::get_logger("stanley_utils"), "Path smoothing failed");
      return smoothed_path;
    }
  }

  // Convert back to Pose
  for (size_t i = 0; i < smoothed_path.size(); i++) {
    smoothed_path.at(i).position.x = x_vector.at(i);
    smoothed_path.at(i).position.y = y_vector.at(i);
  }

  // Recalculate orientation from heading
  for (size_t i = 1; i < static_cast<size_t>(smoothed_path.size()) - 1; ++i) {
    const double dx = smoothed_path[(i + 1)].position.x - smoothed_path[(i - 1)].position.x;
    const double dy = smoothed_path[(i + 1)].position.y - smoothed_path[(i - 1)].position.y;
    double heading = is_forward ? std::atan2(dy, dx) : std::atan2(dy, dx) + M_PI;
    smoothed_path[(i)].orientation = tier4_autoware_utils::createQuaternionFromYaw(heading);
  }
  if (smoothed_path.size() > 1) {
    smoothed_path[0].orientation = smoothed_path[1].orientation;
    smoothed_path.back().orientation = smoothed_path[smoothed_path.size() - 2].orientation;
  }

  return smoothed_path;
}

}  // namespace utils
}  // namespace stanley
}  // namespace autoware

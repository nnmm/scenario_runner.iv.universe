// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scenario_api_utils/scenario_api_utils.hpp"

double normalizeRadian(const double rad, const double min_rad, const double max_rad)
{
  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad < value && value <= max_rad)
    return value;
  else
    return value - std::copysign(2 * M_PI, value);
}

geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quat);
}

double yawFromQuat(double q_x, double q_y, double q_z, double q_w)
{
  tf2::Quaternion quat(q_x, q_y, q_z, q_w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  // cut roll/pitch information
  return yaw;
}

double yawFromQuat(geometry_msgs::msg::Quaternion q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  // cut roll/pitch information
  return yaw;
}

geometry_msgs::msg::Pose poseFromValue(
  const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
  const double o_z, const double o_w)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  pose.orientation.x = o_x;
  pose.orientation.y = o_y;
  pose.orientation.z = o_z;
  pose.orientation.w = o_w;
  return pose;
}

geometry_msgs::msg::Pose poseFromValue(
  const double p_x, const double p_y, const double p_z, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  pose.orientation = quatFromYaw(yaw);
  return pose;
}

geometry_msgs::msg::Pose movePose(const geometry_msgs::msg::Pose & pose, const double move_dist_to_forward)
{
  geometry_msgs::msg::Pose move_pose = pose;
  const double yaw = tf2::getYaw(pose.orientation);
  const double dx = move_dist_to_forward * std::cos(yaw);
  const double dy = move_dist_to_forward * std::sin(yaw);
  move_pose.position.x += dx;
  move_pose.position.y += dy;
  return move_pose;
}

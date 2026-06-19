// Copyright 2018-2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_utils.hpp"

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/normalization.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
namespace
{
double calcLongitudinalOffset(
  const geometry_msgs::msg::Point & p_front, const geometry_msgs::msg::Point & p_back,
  const geometry_msgs::msg::Point & p_target)
{
  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

  return segment_vec.dot(target_vec) / segment_vec.norm();
}
}  // namespace

namespace MPCUtils
{
using autoware_utils::calc_distance2d;
using autoware_utils::create_quaternion_from_yaw;
using autoware_utils::normalize_radian;
using ContinuousTrajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

TrajectoryPoint toTrajectoryPoint(
  const MPCTrajectory & input, const size_t idx, const double wheelbase = 0.0)
{
  TrajectoryPoint p;
  p.pose.position.x = input.x.at(idx);
  p.pose.position.y = input.y.at(idx);
  p.pose.position.z = input.z.at(idx);
  p.pose.orientation = create_quaternion_from_yaw(input.yaw.at(idx));
  p.longitudinal_velocity_mps =
    static_cast<decltype(p.longitudinal_velocity_mps)>(input.vx.at(idx));
  if (!input.relative_time.empty()) {
    p.time_from_start =
      rclcpp::Duration::from_seconds(input.relative_time.at(idx) - input.relative_time.front());
  }
  if (wheelbase != 0.0 && idx < input.smooth_k.size()) {
    p.front_wheel_angle_rad = static_cast<float>(std::atan(input.smooth_k.at(idx) * wheelbase));
  }
  return p;
}

std::vector<TrajectoryPoint> toTrajectoryPoints(
  const MPCTrajectory & input, const double wheelbase = 0.0)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    points.push_back(toTrajectoryPoint(input, i, wheelbase));
  }
  return points;
}

auto buildContinuousTrajectory(const MPCTrajectory & input)
{
  return ContinuousTrajectory::Builder{}.build(toTrajectoryPoints(input));
}

std::vector<double> calcArcLength2d(const MPCTrajectory & trajectory)
{
  std::vector<double> arc_length;
  if (trajectory.empty()) {
    return arc_length;
  }

  double dist = 0.0;
  arc_length.push_back(dist);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const double dx = trajectory.x.at(i) - trajectory.x.at(i - 1);
    const double dy = trajectory.y.at(i) - trajectory.y.at(i - 1);
    dist += std::hypot(dx, dy);
    arc_length.push_back(dist);
  }
  return arc_length;
}

std::vector<double> calcArcLength3d(const MPCTrajectory & trajectory)
{
  std::vector<double> arc_length;
  if (trajectory.empty()) {
    return arc_length;
  }

  double dist = 0.0;
  arc_length.push_back(dist);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const double dx = trajectory.x.at(i) - trajectory.x.at(i - 1);
    const double dy = trajectory.y.at(i) - trajectory.y.at(i - 1);
    const double dz = trajectory.z.at(i) - trajectory.z.at(i - 1);
    dist += std::hypot(std::hypot(dx, dy), dz);
    arc_length.push_back(dist);
  }
  return arc_length;
}

void pushContinuousPoint(
  MPCTrajectory & output, const TrajectoryPoint & p, const double k, const double smooth_k,
  const double relative_time)
{
  output.push_back(
    p.pose.position.x, p.pose.position.y, p.pose.position.z, tf2::getYaw(p.pose.orientation),
    p.longitudinal_velocity_mps, k, smooth_k, relative_time);
}

double calcDistance2d(const MPCTrajectory & trajectory, const size_t idx1, const size_t idx2)
{
  const auto arclength = calcArcLength2d(trajectory);
  const auto continuous_trajectory = buildContinuousTrajectory(trajectory);
  if (continuous_trajectory) {
    const double trajectory_length = continuous_trajectory->length();
    const auto p1 =
      continuous_trajectory->compute(std::clamp(arclength.at(idx1), 0.0, trajectory_length));
    const auto p2 =
      continuous_trajectory->compute(std::clamp(arclength.at(idx2), 0.0, trajectory_length));
    return calc_distance2d(p1, p2);
  }

  const double dx = trajectory.x.at(idx1) - trajectory.x.at(idx2);
  const double dy = trajectory.y.at(idx1) - trajectory.y.at(idx2);
  return std::hypot(dx, dy);
}

double calcDistance3d(const MPCTrajectory & trajectory, const size_t idx1, const size_t idx2)
{
  const auto arclength = calcArcLength2d(trajectory);
  const auto continuous_trajectory = buildContinuousTrajectory(trajectory);
  if (continuous_trajectory) {
    const double trajectory_length = continuous_trajectory->length();
    const auto p1 =
      continuous_trajectory->compute(std::clamp(arclength.at(idx1), 0.0, trajectory_length));
    const auto p2 =
      continuous_trajectory->compute(std::clamp(arclength.at(idx2), 0.0, trajectory_length));
    const double dx = p1.pose.position.x - p2.pose.position.x;
    const double dy = p1.pose.position.y - p2.pose.position.y;
    const double dz = p1.pose.position.z - p2.pose.position.z;
    return std::hypot(std::hypot(dx, dy), dz);
  }

  const double dx = trajectory.x.at(idx1) - trajectory.x.at(idx2);
  const double dy = trajectory.y.at(idx1) - trajectory.y.at(idx2);
  const double dz = trajectory.z.at(idx1) - trajectory.z.at(idx2);
  return std::hypot(dx, dy, dz);
}

void convertEulerAngleToMonotonic(std::vector<double> & angle_vector)
{
  for (uint i = 1; i < angle_vector.size(); ++i) {
    const double da = angle_vector.at(i) - angle_vector.at(i - 1);
    angle_vector.at(i) = angle_vector.at(i - 1) + normalize_radian(da);
  }
}

double calcLateralError(const Pose & ego_pose, const Pose & ref_pose)
{
  const double err_x = ego_pose.position.x - ref_pose.position.x;
  const double err_y = ego_pose.position.y - ref_pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

void calcMPCTrajectoryArcLength(const MPCTrajectory & trajectory, std::vector<double> & arc_length)
{
  arc_length = calcArcLength2d(trajectory);
}

double calcMPCTrajectoryArcLength(const MPCTrajectory & trajectory)
{
  const auto arc_length = calcArcLength2d(trajectory);
  return arc_length.empty() ? 0.0 : arc_length.back();
}

std::pair<bool, MPCTrajectory> resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, const size_t nearest_seg_idx,
  const double ego_offset_to_segment)
{
  MPCTrajectory output;

  if (input.empty()) {
    return {true, output};
  }
  if (resample_interval_dist <= std::numeric_limits<double>::epsilon()) {
    return {false, output};
  }

  std::vector<double> input_arclength;
  calcMPCTrajectoryArcLength(input, input_arclength);

  if (input_arclength.empty() || nearest_seg_idx >= input_arclength.size()) {
    return {false, output};
  }

  const auto continuous_trajectory = buildContinuousTrajectory(input);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    return {false, output};
  }

  const double trajectory_length = continuous_trajectory->length();
  if (trajectory_length <= std::numeric_limits<double>::epsilon()) {
    return {false, output};
  }

  std::vector<double> output_arclength;
  // To accurately sample the ego point, resample separately in the forward direction and the
  // backward direction from the current position.
  const double origin_s = input_arclength.at(nearest_seg_idx) + ego_offset_to_segment;
  for (double s = std::clamp(origin_s, 0.0, trajectory_length - 1e-6); 0 <= s;
       s -= resample_interval_dist) {
    output_arclength.push_back(s);
  }
  std::reverse(output_arclength.begin(), output_arclength.end());
  for (double s = std::max(origin_s, 0.0) + resample_interval_dist; s < trajectory_length;
       s += resample_interval_dist) {
    output_arclength.push_back(s);
  }

  const auto lerp_arc_length = [&](const auto & input_value) {
    return autoware::interpolation::lerp(input_arclength, input_value, output_arclength);
  };
  const auto spline_arc_length = [&](const auto & input_value) {
    return autoware::interpolation::spline(input_arclength, input_value, output_arclength);
  };

  const auto output_k = spline_arc_length(input.k);
  const auto output_smooth_k = spline_arc_length(input.smooth_k);
  const auto output_relative_time = lerp_arc_length(input.relative_time);  // must be linear

  for (size_t i = 0; i < output_arclength.size(); ++i) {
    pushContinuousPoint(
      output, continuous_trajectory->compute(output_arclength.at(i)), output_k.at(i),
      output_smooth_k.at(i), output_relative_time.at(i));
  }

  return {true, output};
}

bool linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory & out_traj)
{
  if (in_traj.empty()) {
    out_traj = in_traj;
    return true;
  }
  out_traj.clear();

  const auto lerp_arc_length = [&](const auto & input_value) {
    return autoware::interpolation::lerp(in_index, input_value, out_index);
  };

  try {
    const auto output_k = lerp_arc_length(in_traj.k);
    const auto output_smooth_k = lerp_arc_length(in_traj.smooth_k);
    const auto output_relative_time = lerp_arc_length(in_traj.relative_time);

    const auto input_arclength = calcArcLength2d(in_traj);
    const auto output_arclength =
      autoware::interpolation::lerp(in_index, input_arclength, out_index);
    const auto continuous_trajectory = buildContinuousTrajectory(in_traj);
    if (!continuous_trajectory) {
      std::cerr << "[mpc util] failed to build continuous trajectory: "
                << continuous_trajectory.error().what << std::endl;
      return false;
    }
    const double trajectory_length = continuous_trajectory->length();
    for (size_t i = 0; i < output_arclength.size(); ++i) {
      const double s = std::clamp(output_arclength.at(i), 0.0, trajectory_length);
      pushContinuousPoint(
        out_traj, continuous_trajectory->compute(s), output_k.at(i), output_smooth_k.at(i),
        output_relative_time.at(i));
    }
  } catch (const std::exception & e) {
    std::cerr << "linearInterpMPCTrajectory error!: " << e.what() << std::endl;
  }

  if (out_traj.empty()) {
    std::cerr << "[mpc util] linear interpolation error" << std::endl;
    return false;
  }

  return true;
}

void calcTrajectoryYawFromXY(MPCTrajectory & traj, const bool is_forward_shift)
{
  if (traj.yaw.size() < 3) {  // at least 3 points are required to calculate yaw
    return;
  }
  if (traj.yaw.size() != traj.vx.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc_utils"), "trajectory size has no consistency.");
    return;
  }

  auto continuous_trajectory = buildContinuousTrajectory(traj);
  if (!continuous_trajectory) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc_utils"), "failed to build continuous trajectory.");
    return;
  }

  continuous_trajectory->align_orientation_with_trajectory_direction();
  const auto arclength = calcArcLength2d(traj);
  for (size_t i = 0; i < traj.yaw.size(); ++i) {
    const auto yaw = tf2::getYaw(continuous_trajectory->compute(arclength.at(i)).pose.orientation);
    traj.yaw.at(i) = is_forward_shift ? yaw : yaw + M_PI;
  }
}

void calcTrajectoryCurvature(
  const int curvature_smoothing_num_traj, const int curvature_smoothing_num_ref_steer,
  MPCTrajectory & traj)
{
  traj.k = calcTrajectoryCurvature(curvature_smoothing_num_traj, traj);
  traj.smooth_k = calcTrajectoryCurvature(curvature_smoothing_num_ref_steer, traj);
}

std::vector<double> calcTrajectoryCurvature(
  const int curvature_smoothing_num, const MPCTrajectory & traj)
{
  std::vector<double> curvature_vec(traj.x.size());

  if (traj.size() < 3) {
    return curvature_vec;
  }

  const auto arclength = calcArcLength2d(traj);
  const auto continuous_trajectory = buildContinuousTrajectory(traj);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    return curvature_vec;
  }

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  const int max_smoothing_num =
    static_cast<int>(std::floor(0.5 * (static_cast<double>(traj.x.size() - 1))));
  const size_t L = static_cast<size_t>(std::min(curvature_smoothing_num, max_smoothing_num));
  for (size_t i = L; i < traj.x.size() - L; ++i) {
    const size_t curr_idx = i;
    const size_t prev_idx = curr_idx - L;
    const size_t next_idx = curr_idx + L;
    const double trajectory_length = continuous_trajectory->length();
    p1 = continuous_trajectory->compute(std::clamp(arclength.at(prev_idx), 0.0, trajectory_length))
           .pose.position;
    p2 = continuous_trajectory->compute(std::clamp(arclength.at(curr_idx), 0.0, trajectory_length))
           .pose.position;
    p3 = continuous_trajectory->compute(std::clamp(arclength.at(next_idx), 0.0, trajectory_length))
           .pose.position;
    try {
      curvature_vec.at(curr_idx) = autoware_utils::calc_curvature(p1, p2, p3);
    } catch (...) {
      std::cerr << "[MPC] 2 points are too close to calculate curvature." << std::endl;
      curvature_vec.at(curr_idx) = 0.0;
    }
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, traj.x.size()); ++i) {
    curvature_vec.at(i) = curvature_vec.at(std::min(L, traj.x.size() - 1));
    curvature_vec.at(traj.x.size() - i - 1) =
      curvature_vec.at(std::max(traj.x.size() - L - 1, size_t(0)));
  }
  return curvature_vec;
}

MPCTrajectory convertToMPCTrajectory(const Trajectory & input)
{
  MPCTrajectory output;

  if (input.points.empty()) {
    return output;
  }

  const auto push_back_point = [&](const TrajectoryPoint & p) {
    const double x = p.pose.position.x;
    const double y = p.pose.position.y;
    const double z = p.pose.position.z;
    const double yaw = tf2::getYaw(p.pose.orientation);
    const double vx = p.longitudinal_velocity_mps;
    const double k = 0.0;
    const double t = 0.0;
    output.push_back(x, y, z, yaw, vx, k, k, t);
  };

  if (input.points.size() < 2) {
    for (const TrajectoryPoint & p : input.points) {
      push_back_point(p);
    }
    calcMPCTrajectoryTime(output);
    return output;
  }

  const auto continuous_trajectory = ContinuousTrajectory::Builder{}.build(input.points);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    for (const TrajectoryPoint & p : input.points) {
      push_back_point(p);
    }
    calcMPCTrajectoryTime(output);
    return output;
  }

  std::vector<double> input_arclength;
  input_arclength.reserve(input.points.size());
  input_arclength.push_back(0.0);
  for (size_t i = 1; i < input.points.size(); ++i) {
    input_arclength.push_back(
      input_arclength.back() + calc_distance2d(input.points.at(i - 1), input.points.at(i)));
  }

  const double trajectory_length = continuous_trajectory->length();
  for (const auto s : input_arclength) {
    push_back_point(continuous_trajectory->compute(std::clamp(s, 0.0, trajectory_length)));
  }

  calcMPCTrajectoryTime(output);
  return output;
}

Trajectory convertToAutowareTrajectory(const MPCTrajectory & input, const double wheelbase)
{
  Trajectory output;
  if (input.empty()) {
    return output;
  }

  const auto continuous_trajectory = buildContinuousTrajectory(input);
  const auto arclength = calcArcLength2d(input);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    output.points = toTrajectoryPoints(input, wheelbase);
    return output;
  }

  for (size_t i = 0; i < input.size(); ++i) {
    auto p = continuous_trajectory->compute(
      std::clamp(arclength.at(i), 0.0, continuous_trajectory->length()));
    if (!input.relative_time.empty()) {
      p.time_from_start =
        rclcpp::Duration::from_seconds(input.relative_time.at(i) - input.relative_time.front());
    }
    if (wheelbase != 0.0 && i < input.smooth_k.size()) {
      p.front_wheel_angle_rad = static_cast<float>(std::atan(input.smooth_k.at(i) * wheelbase));
    }
    output.points.push_back(p);
    if (output.points.size() == output.points.max_size()) {
      break;
    }
  }
  return output;
}

bool calcMPCTrajectoryTime(MPCTrajectory & traj)
{
  if (traj.empty()) {
    return true;
  }

  constexpr auto min_dt = 1.0e-4;  // must be positive value to avoid duplication in time
  const auto arclength = calcArcLength3d(traj);
  traj.relative_time.clear();
  traj.relative_time.push_back(0.0);
  for (size_t i = 0; i < traj.x.size() - 1; ++i) {
    const double dist = arclength.at(i + 1) - arclength.at(i);
    const double v = std::max(std::fabs(traj.vx.at(i)), 0.1);
    traj.relative_time.push_back(traj.relative_time.back() + std::max(dist / v, min_dt));
  }
  return true;
}

void dynamicSmoothingVelocity(
  const size_t start_seg_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory & traj)
{
  const auto arclength = calcArcLength2d(traj);
  double curr_v = start_vel;
  // set current velocity in both start and end point of the segment
  traj.vx.at(start_seg_idx) = start_vel;
  if (1 < traj.vx.size()) {
    traj.vx.at(start_seg_idx + 1) = start_vel;
  }

  for (size_t i = start_seg_idx + 2; i < traj.size(); ++i) {
    const double ds = arclength.at(i) - arclength.at(i - 1);
    const double dt = ds / std::max(std::fabs(curr_v), std::numeric_limits<double>::epsilon());
    const double a = tau / std::max(tau + dt, std::numeric_limits<double>::epsilon());
    const double updated_v = a * curr_v + (1.0 - a) * traj.vx.at(i);
    const double dv = std::max(-acc_lim * dt, std::min(acc_lim * dt, updated_v - curr_v));
    curr_v = curr_v + dv;
    traj.vx.at(i) = curr_v;
  }
  calcMPCTrajectoryTime(traj);
}

bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const Pose & self_pose, Pose * nearest_pose, size_t * nearest_index,
  double * nearest_time, const double max_dist, const double max_yaw)
{
  if (traj.empty() || !nearest_pose || !nearest_index || !nearest_time) {
    return false;
  }

  const auto autoware_traj = convertToAutowareTrajectory(traj);
  if (autoware_traj.points.empty()) {
    const auto logger = rclcpp::get_logger("mpc_util");
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(logger, clock, 5000, "[calcNearestPoseInterp] input trajectory is empty");
    return false;
  }

  *nearest_index = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, self_pose, max_dist, max_yaw);
  const size_t traj_size = traj.size();
  const auto arclength = calcArcLength2d(traj);
  const auto continuous_trajectory = buildContinuousTrajectory(traj);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    return false;
  }

  if (traj.size() == 1) {
    *nearest_pose =
      continuous_trajectory
        ->compute(std::clamp(arclength.at(*nearest_index), 0.0, continuous_trajectory->length()))
        .pose;
    *nearest_time = traj.relative_time.at(*nearest_index);
    return true;
  }

  /* get second nearest index = next to nearest_index */
  const auto [prev, next] = [&]() -> std::pair<size_t, size_t> {
    if (*nearest_index == 0) {
      return std::make_pair(0, 1);
    }
    if (*nearest_index == traj_size - 1) {
      return std::make_pair(traj_size - 2, traj_size - 1);
    }

    geometry_msgs::msg::Point nearest_traj_point;
    nearest_traj_point.x = traj.x.at(*nearest_index);
    nearest_traj_point.y = traj.y.at(*nearest_index);
    geometry_msgs::msg::Point next_nearest_traj_point;
    next_nearest_traj_point.x = traj.x.at(*nearest_index + 1);
    next_nearest_traj_point.y = traj.y.at(*nearest_index + 1);

    const double signed_length =
      calcLongitudinalOffset(nearest_traj_point, next_nearest_traj_point, self_pose.position);
    if (signed_length <= 0) {
      return std::make_pair(*nearest_index - 1, *nearest_index);
    }
    return std::make_pair(*nearest_index, *nearest_index + 1);
  }();

  geometry_msgs::msg::Point next_traj_point;
  next_traj_point.x = traj.x.at(next);
  next_traj_point.y = traj.y.at(next);
  geometry_msgs::msg::Point prev_traj_point;
  prev_traj_point.x = traj.x.at(prev);
  prev_traj_point.y = traj.y.at(prev);
  const double traj_seg_length = autoware_utils::calc_distance2d(prev_traj_point, next_traj_point);
  /* if distance between two points are too close */
  if (traj_seg_length < 1.0E-5) {
    *nearest_pose =
      continuous_trajectory
        ->compute(std::clamp(arclength.at(*nearest_index), 0.0, continuous_trajectory->length()))
        .pose;
    *nearest_time = traj.relative_time.at(*nearest_index);
    return true;
  }

  /* linear interpolation */
  const double ratio = std::clamp(
    calcLongitudinalOffset(prev_traj_point, next_traj_point, self_pose.position) / traj_seg_length,
    0.0, 1.0);
  const double nearest_s = (1 - ratio) * arclength.at(prev) + ratio * arclength.at(next);
  *nearest_pose =
    continuous_trajectory->compute(std::clamp(nearest_s, 0.0, continuous_trajectory->length()))
      .pose;
  *nearest_time = (1 - ratio) * traj.relative_time.at(prev) + ratio * traj.relative_time.at(next);
  return true;
}

double calcStopDistance(const Trajectory & current_trajectory, const int origin)
{
  constexpr float zero_velocity = std::numeric_limits<float>::epsilon();
  const float origin_velocity =
    current_trajectory.points.at(static_cast<size_t>(origin)).longitudinal_velocity_mps;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < static_cast<int>(current_trajectory.points.size()) - 1; ++i) {
      const auto & p0 = current_trajectory.points.at(i);
      const auto & p1 = current_trajectory.points.at(i - 1);
      stop_dist += calc_distance2d(p0, p1);
      if (std::fabs(p0.longitudinal_velocity_mps) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    const auto & p0 = current_trajectory.points.at(i);
    const auto & p1 = current_trajectory.points.at(i + 1);
    if (std::fabs(p0.longitudinal_velocity_mps) > zero_velocity) {
      break;
    }
    stop_dist -= calc_distance2d(p0, p1);
  }
  return stop_dist;
}

void extendTrajectoryInYawDirection(
  const double yaw, const double interval, const bool is_forward_shift, MPCTrajectory & traj)
{
  if (traj.empty()) return;

  // set terminal yaw
  traj.yaw.back() = yaw;

  // get terminal pose
  const auto continuous_trajectory = buildContinuousTrajectory(traj);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    return;
  }
  auto extended_pose = continuous_trajectory->compute(continuous_trajectory->length()).pose;

  constexpr double extend_dist = 10.0;
  const double extend_vel = traj.vx.back();
  const double x_offset = is_forward_shift ? interval : -interval;
  constexpr double min_vel_threshold = 0.1;
  const double dt =
    (std::fabs(extend_vel) < min_vel_threshold) ? 1.0e-4 : interval / std::fabs(extend_vel);
  const size_t num_extended_point = static_cast<size_t>(extend_dist / interval);
  for (size_t i = 0; i < num_extended_point; ++i) {
    extended_pose = autoware_utils::calc_offset_pose(extended_pose, x_offset, 0.0, 0.0);
    traj.push_back(
      extended_pose.position.x, extended_pose.position.y, extended_pose.position.z, traj.yaw.back(),
      extend_vel, traj.k.back(), traj.smooth_k.back(), traj.relative_time.back() + dt);
  }
}

MPCTrajectory clipTrajectoryByLength(const MPCTrajectory & trajectory, const double length)
{
  MPCTrajectory clipped_trajectory;
  if (trajectory.empty()) {
    return clipped_trajectory;
  }

  const auto continuous_trajectory = buildContinuousTrajectory(trajectory);
  if (!continuous_trajectory) {
    std::cerr << "[mpc util] failed to build continuous trajectory: "
              << continuous_trajectory.error().what << std::endl;
    return clipped_trajectory;
  }

  const auto arclength = calcArcLength2d(trajectory);
  pushContinuousPoint(
    clipped_trajectory, continuous_trajectory->compute(arclength.front()), trajectory.k.front(),
    trajectory.smooth_k.front(), trajectory.relative_time.front());

  for (size_t i = 1; i < trajectory.size(); ++i) {
    if (arclength.at(i) > length) {
      break;
    }
    pushContinuousPoint(
      clipped_trajectory, continuous_trajectory->compute(arclength.at(i)), trajectory.k.at(i),
      trajectory.smooth_k.at(i), trajectory.relative_time.at(i));
  }

  return clipped_trajectory;
}

}  // namespace MPCUtils
}  // namespace autoware::motion::control::mpc_lateral_controller

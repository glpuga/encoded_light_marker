/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// standard library
#include <cmath>

// project
#include <elm_trackers/ToroidalTrackerDriver.hpp>

namespace elm_trackers {

ToroidalTrackerDriver::ToroidalTrackerDriver(const size_t width,
                                             const size_t height,
                                             const double orientation,
                                             const double speed)
    : width_{width},
      height_{height},
      orientation_{orientation},
      speed_{speed},
      current_pose_x_{static_cast<double>(width / 2)},
      current_pose_y_{static_cast<double>(height / 2)} {}

Coordinates2D ToroidalTrackerDriver::evaluateCurrentPosition(
    const Time &curr_time) {
  if (!initialized_) {
    initialized_ = true;
    latest_timestamp_ = curr_time;
  }

  const auto velocity_vector_x = speed_ * std::cos(orientation_);
  const auto velocity_vector_y = speed_ * std::sin(orientation_);

  const auto delta_t_ns =
      (curr_time.ns() - latest_timestamp_.ns()) % 1000000000;
  const auto delta_t_fp = static_cast<double>(delta_t_ns) / 1.0e9;

  latest_timestamp_ = curr_time;

  current_pose_x_ += velocity_vector_x * delta_t_fp;
  current_pose_y_ += velocity_vector_y * delta_t_fp;

  if (current_pose_x_ < 0.0) {
    current_pose_x_ += width_;
  }
  if (current_pose_x_ >= width_) {
    current_pose_x_ -= width_;
  }

  if (current_pose_y_ < 0.0) {
    current_pose_y_ += height_;
  }
  if (current_pose_y_ >= height_) {
    current_pose_y_ -= height_;
  }

  Coordinates2D coordinates;
  coordinates.x = static_cast<int32_t>(current_pose_x_);
  coordinates.y = static_cast<int32_t>(current_pose_y_);
  return coordinates;
}

}  // namespace elm_trackers
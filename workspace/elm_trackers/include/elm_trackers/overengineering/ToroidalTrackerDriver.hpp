/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// project
#include <elm_trackers/overengineering/TrackerDriverInterface.hpp>

namespace elm_trackers {

class ToroidalTrackerDriver : public TrackerDriverInterface {
 public:
  ToroidalTrackerDriver(const size_t width, const size_t height,
                        const double orientation, const double speed);

  Coordinates2D evaluateCurrentPosition(const Time &curr_time) override;

 private:
  bool initialized_{false};
  Time latest_timestamp_;

  size_t width_;
  size_t height_;
  double orientation_;
  double speed_;

  double current_pose_x_;
  double current_pose_y_;
};

}  // namespace elm_trackers
/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <memory>

// project
#include <elm_trackers/datatypes/Coordinates2D.hpp>
#include <elm_trackers/datatypes/Time.hpp>

namespace elm_trackers {

class TrackerDriverInterface {
 public:
  using Ptr = std::unique_ptr<TrackerDriverInterface>;
  using SharedPtr = std::shared_ptr<TrackerDriverInterface>;

  virtual ~TrackerDriverInterface() = default;

  virtual Coordinates2D evaluateCurrentPosition(const Time &curr_time) = 0;
};

}  // namespace elm_trackers
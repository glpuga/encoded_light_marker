/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <memory>

// project
#include <elm_trackers/datatypes/Coordinates2D.hpp>
#include <elm_trackers/datatypes/ModulationState.hpp>

namespace elm_trackers {

class TrackerDrawerInterface {
 public:
  using Ptr = std::unique_ptr<TrackerDrawerInterface>;
  using SharedPtr = std::shared_ptr<TrackerDrawerInterface>;

  virtual ~TrackerDrawerInterface() = default;

  virtual void draw(const Coordinates2D &, const ModulationState &) = 0;
};

}  // namespace elm_trackers

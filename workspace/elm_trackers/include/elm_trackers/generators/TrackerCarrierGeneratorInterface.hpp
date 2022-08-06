/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <memory>

// project
#include <elm_trackers/datatypes/ModulationState.hpp>
#include <elm_trackers/datatypes/Time.hpp>

namespace elm_trackers {

class TrackerCarrierGeneratorInterface {
 public:
  using Ptr = std::unique_ptr<TrackerCarrierGeneratorInterface>;
  using SharedPtr = std::shared_ptr<TrackerCarrierGeneratorInterface>;

  virtual ~TrackerCarrierGeneratorInterface() = default;

  virtual ModulationState evaluateModulationState(const Time &curr_time) = 0;
};

}  // namespace elm_trackers

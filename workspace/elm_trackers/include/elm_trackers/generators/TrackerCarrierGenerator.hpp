/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <chrono>

// project
#include <elm_trackers/generators/TrackerCarrierGeneratorInterface.hpp>
#include <elm_trackers/datatypes/TrackerId.hpp>

namespace elm_trackers {

class TrackerCarrierGenerator : public TrackerCarrierGeneratorInterface {
 public:
  TrackerCarrierGenerator(const TrackerId id, const double rate);

  ModulationState evaluateModulationState(const Time &curr_time) override;

 private:
  bool initialized_{false};
  Time start_time_;

  TrackerId id_;
  double rate_;
};

}  // namespace elm_trackers
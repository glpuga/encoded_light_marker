/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <chrono>

// project
#include <elm_trackers/datatypes/TrackerId.hpp>
#include <elm_trackers/generators/TrackerCarrierGeneratorInterface.hpp>

namespace elm_trackers {

class TrackerCarrierGenerator : public TrackerCarrierGeneratorInterface {
 public:
  TrackerCarrierGenerator(const double rate)
      : TrackerCarrierGenerator(rate, TrackerId{}){};

  TrackerCarrierGenerator(const double rate, const TrackerId id);

  ModulationState evaluateModulationState(const Time &curr_time) override;

 private:
  bool initialized_{false};
  Time start_time_;

  TrackerId id_;
  double rate_;
};

}  // namespace elm_trackers

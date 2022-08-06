/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// standard library
#include <math.h>

// project
#include <elm_trackers/generators/TrackerCarrierGenerator.hpp>

namespace elm_trackers {

TrackerCarrierGenerator::TrackerCarrierGenerator(const TrackerId id,
                                                 const double rate)
    : id_{id}, rate_{rate} {}

ModulationState TrackerCarrierGenerator::evaluateModulationState(
    const Time &curr_time) {
  if (!initialized_) {
    initialized_ = true;
    start_time_ = curr_time;
  }

  const auto delta_t_ns = (curr_time.ns() - start_time_.ns()) % 1000000000;
  const auto delta_t_fp = static_cast<double>(delta_t_ns) / 1.0e9;

  CarrierPhase carrier_phase;
  carrier_phase.phi = 2 * M_PI * rate_ * delta_t_fp;

  ModulationState modulation_state;
  modulation_state.carrier_phase = carrier_phase;

  return modulation_state;
}

}  // namespace elm_trackers

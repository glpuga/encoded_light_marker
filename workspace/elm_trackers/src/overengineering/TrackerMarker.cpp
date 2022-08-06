/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// standard library
#include <cmath>
#include <utility>

// project
#include <elm_trackers/datatypes/Time.hpp>
#include <elm_trackers/overengineering/TrackerMarker.hpp>

namespace elm_trackers {

TrackerMarker::TrackerMarker(
    TrackerCarrierGeneratorInterface::Ptr carrier_generator,
    TrackerDriverInterface::Ptr driver, TrackerDrawerInterface::Ptr drawer)
    : carrier_generator_{std::move(carrier_generator)},
      driver_{std::move(driver)},
      drawer_{std::move(drawer)} {}

void TrackerMarker::draw() {
  Time curr_time;
  const auto modulation_state =
      carrier_generator_->evaluateModulationState(curr_time);
  const auto position_2d = driver_->evaluateCurrentPosition(curr_time);
  drawer_->draw(position_2d, modulation_state);
}

}  // namespace elm_trackers

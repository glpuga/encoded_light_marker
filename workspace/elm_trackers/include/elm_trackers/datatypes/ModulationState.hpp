/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// project
#include <elm_trackers/datatypes/CarrierPhase.hpp>

namespace elm_trackers {

struct ModulationState {
  CarrierPhase carrier_phase;
};

}  // namespace elm_trackers

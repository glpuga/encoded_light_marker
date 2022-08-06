/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// project
#include <elm_trackers/datatypes/Buffer2D.hpp>
#include <elm_trackers/datatypes/Time.hpp>
#include <elm_trackers/generators/TrackerCarrierGeneratorInterface.hpp>

namespace elm_trackers {

class IQCorrelatorBufferInterface {
 public:
  using Ptr = std::unique_ptr<IQCorrelatorBufferInterface>;

  virtual ~IQCorrelatorBufferInterface() = default;

  virtual void reset() = 0;

  virtual void feedSample(const Time &timestamp, const Buffer2D &sample) = 0;

  virtual const Buffer2D &phaseIAccumulator() const = 0;
  virtual const Buffer2D &phaseQAccumulator() const = 0;

  virtual Buffer2D squaredModule() const = 0;
};

}  // namespace elm_trackers

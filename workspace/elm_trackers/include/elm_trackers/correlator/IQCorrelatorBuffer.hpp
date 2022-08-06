/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// project
#include <elm_trackers/correlator/IQCorrelatorBufferInterface.hpp>
#include <elm_trackers/generators/TrackerCarrierGeneratorInterface.hpp>

namespace elm_trackers {

class IQCorrelatorBuffer : public IQCorrelatorBufferInterface {
public:
  IQCorrelatorBuffer(const std::size_t width, const std::size_t height,
                     TrackerCarrierGeneratorInterface::Ptr generator);

  void reset() override;

  void feedSample(const Time &timestamp, const Buffer2D &sample) override;

  const Buffer2D &phaseIAccumulator() const override;
  const Buffer2D &phaseQAccumulator() const override;

  Buffer2D squaredModule() const override;

private:
  Buffer2D accumulator_buffer_i_;
  Buffer2D accumulator_buffer_q_;
  TrackerCarrierGeneratorInterface::Ptr generator_;
};

} // namespace elm_trackers

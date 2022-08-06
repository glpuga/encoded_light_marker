/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// standard library
#include <cmath>
#include <stdexcept>
#include <utility>

// project
#include <elm_trackers/correlator/IQCorrelatorBuffer.hpp>

namespace elm_trackers {

IQCorrelatorBuffer::IQCorrelatorBuffer(
    const std::size_t width, const std::size_t height,
    TrackerCarrierGeneratorInterface::Ptr generator)
    : accumulator_buffer_i_{width, height},
      accumulator_buffer_q_{width, height}, generator_{std::move(generator)} {}

void IQCorrelatorBuffer::reset() {
  accumulator_buffer_i_.reset();
  accumulator_buffer_q_.reset();
}

void IQCorrelatorBuffer::feedSample(const Time &timestamp,
                                    const Buffer2D &sample) {
  if ((sample.width() != accumulator_buffer_i_.width()) ||
      (sample.height() != accumulator_buffer_i_.height())) {
    throw std::logic_error{
        "Sample buffer does not match accumultor buffer size"};
  }

  const auto carrier_phase = generator_->evaluateModulationState(timestamp);

  const auto carrier_i = std::cos(carrier_phase.carrier_phase.phi);
  const auto carrier_q = std::sin(carrier_phase.carrier_phase.phi);

  Buffer2D modulated_sample = sample;
  modulated_sample *= carrier_i;
  accumulator_buffer_i_ += modulated_sample;

  modulated_sample = sample;
  modulated_sample *= carrier_q;
  accumulator_buffer_q_ += modulated_sample;
}

const Buffer2D &IQCorrelatorBuffer::phaseIAccumulator() const {
  return accumulator_buffer_i_;
}

const Buffer2D &IQCorrelatorBuffer::phaseQAccumulator() const {
  return accumulator_buffer_q_;
}

Buffer2D IQCorrelatorBuffer::squaredModule() const {
  Buffer2D squared_module_i{accumulator_buffer_i_};
  squared_module_i *= accumulator_buffer_i_;

  Buffer2D squared_module_q{accumulator_buffer_q_};
  squared_module_q *= accumulator_buffer_q_;

  squared_module_i += squared_module_q;
  return squared_module_i;
}

} // namespace elm_trackers

// Copyright (2022) Gerardo Puga
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

// gtest
#include "gtest/gtest.h"

// project
#include <elm_trackers/correlator/IQCorrelatorBuffer.hpp>
#include <elm_trackers/datatypes/Time.hpp>
#include <elm_trackers/generators/TrackerCarrierGenerator.hpp>

namespace elm_trackers {

namespace test {

namespace {

using ::testing::Test;

struct IQCorrelatorBufferTests : public Test {
  const double rate_ = 1.0;
  const double sample_interval_ = 0.001;
  const double correlation_interval_ = 10.0;

  IQCorrelatorBufferInterface::Ptr uut_;

  IQCorrelatorBufferTests() {
    auto generator = std::make_unique<TrackerCarrierGenerator>(rate_);
    uut_ = std::make_unique<IQCorrelatorBuffer>(2, 2, std::move(generator));
  }
};

TEST_F(IQCorrelatorBufferTests, ConstructionTest) {
  Buffer2D input{2, 2};

  for (double t = 0.0; t < correlation_interval_; t += sample_interval_) {
    input.cell(0, 0) = std::cos(2 * M_PI * rate_ * t);
    input.cell(1, 0) = std::sin(2 * M_PI * rate_ * t);
    input.cell(0, 1) = -std::cos(2 * M_PI * rate_ * t);
    input.cell(1, 1) = -std::sin(2 * M_PI * rate_ * t);

    uut_->feedSample(
        Time(std::chrono::milliseconds(static_cast<int>(t * 1000))), input);
  }

  // scale to get correlator output to mean output value
  const double scale = sample_interval_ / correlation_interval_;
  const double squared_scale = scale * scale;

  {
    const auto i_acc = uut_->phaseIAccumulator();
    const auto q_acc = uut_->phaseQAccumulator();
    const auto sm_acc = uut_->squaredModule();

    EXPECT_NEAR(scale * i_acc.cell(0, 0), 0.5, 0.01);
    EXPECT_NEAR(scale * i_acc.cell(1, 0), 0.0, 0.01);
    EXPECT_NEAR(scale * i_acc.cell(0, 1), -0.5, 0.01);
    EXPECT_NEAR(scale * i_acc.cell(1, 1), 0.0, 0.01);

    EXPECT_NEAR(scale * q_acc.cell(0, 0), 0.0, 0.01);
    EXPECT_NEAR(scale * q_acc.cell(1, 0), 0.5, 0.01);
    EXPECT_NEAR(scale * q_acc.cell(0, 1), 0.0, 0.01);
    EXPECT_NEAR(scale * q_acc.cell(1, 1), -0.5, 0.01);

    EXPECT_NEAR(squared_scale * sm_acc.cell(0, 0), 0.25, 0.03);
    EXPECT_NEAR(squared_scale * sm_acc.cell(1, 0), 0.25, 0.03);
    EXPECT_NEAR(squared_scale * sm_acc.cell(0, 1), 0.25, 0.03);
    EXPECT_NEAR(squared_scale * sm_acc.cell(1, 1), 0.25, 0.03);
  }

  uut_->reset();

  {
    const auto i_acc = uut_->phaseIAccumulator();
    const auto q_acc = uut_->phaseQAccumulator();
    const auto sm_acc = uut_->squaredModule();

    EXPECT_NEAR(scale * i_acc.cell(0, 0), 0.0, 0.01);
    EXPECT_NEAR(scale * i_acc.cell(1, 0), 0.0, 0.01);
    EXPECT_NEAR(scale * i_acc.cell(0, 1), 0.0, 0.01);
    EXPECT_NEAR(scale * i_acc.cell(1, 1), 0.0, 0.01);

    EXPECT_NEAR(scale * q_acc.cell(0, 0), 0.0, 0.01);
    EXPECT_NEAR(scale * q_acc.cell(1, 0), 0.0, 0.01);
    EXPECT_NEAR(scale * q_acc.cell(0, 1), 0.0, 0.01);
    EXPECT_NEAR(scale * q_acc.cell(1, 1), 0.0, 0.01);

    EXPECT_NEAR(squared_scale * sm_acc.cell(0, 0), 0.0, 0.03);
    EXPECT_NEAR(squared_scale * sm_acc.cell(1, 0), 0.0, 0.03);
    EXPECT_NEAR(squared_scale * sm_acc.cell(0, 1), 0.0, 0.03);
    EXPECT_NEAR(squared_scale * sm_acc.cell(1, 1), 0.0, 0.03);
  }
}

} // namespace

} // namespace test

} // namespace elm_trackers

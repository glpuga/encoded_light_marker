/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <chrono>

namespace elm_trackers {

struct Time {
public:
  Time() = default;

  Time(const std::chrono::nanoseconds &timestamp) : timestamp_{timestamp} {}

  std::chrono::nanoseconds &value() { return timestamp_; }

  const std::chrono::nanoseconds &value() const { return timestamp_; }

  std::chrono::nanoseconds::rep ns() const { return timestamp_.count(); }

private:
  std::chrono::nanoseconds timestamp_;
};

} // namespace elm_trackers

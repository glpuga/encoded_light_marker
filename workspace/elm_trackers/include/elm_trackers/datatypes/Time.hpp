/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <chrono>

namespace elm_trackers {

struct Time {
  std::chrono::nanoseconds timestamp;

  Time() = default;

  std::chrono::nanoseconds::rep ns() const { return timestamp.count(); }
};

}  // namespace elm_trackers

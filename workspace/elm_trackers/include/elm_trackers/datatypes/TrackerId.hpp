/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <cstdint>
#include <memory>

namespace elm_trackers {

class TrackerId {
 public:
  TrackerId(const size_t id) : id_{id} {}

  size_t id() const { return id_; }

 private:
  size_t id_;
};

}  // namespace elm_trackers

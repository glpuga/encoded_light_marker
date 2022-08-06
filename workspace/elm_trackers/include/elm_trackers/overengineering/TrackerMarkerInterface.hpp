/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

namespace elm_trackers {

class TrackerMarkerInterface {
public:
  virtual ~TrackerMarkerInterface() = default;

  virtual void draw() = 0;
};

} // namespace elm_trackers

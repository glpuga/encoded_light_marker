/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <utility>

// project
#include <elm_trackers/generators/TrackerCarrierGeneratorInterface.hpp>
#include <elm_trackers/overengineering/TrackerDrawerInterface.hpp>
#include <elm_trackers/overengineering/TrackerDriverInterface.hpp>
#include <elm_trackers/overengineering/TrackerMarkerInterface.hpp>

namespace elm_trackers {

class TrackerMarker : public TrackerMarkerInterface {
public:
  TrackerMarker(TrackerCarrierGeneratorInterface::Ptr carrier_generator,
                TrackerDriverInterface::Ptr driver,
                TrackerDrawerInterface::Ptr drawer);

  void draw();

private:
  TrackerCarrierGeneratorInterface::Ptr carrier_generator_;
  TrackerDriverInterface::Ptr driver_;
  TrackerDrawerInterface::Ptr drawer_;
};

} // namespace elm_trackers

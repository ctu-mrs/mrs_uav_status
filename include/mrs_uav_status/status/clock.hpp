#pragma once

/* includes //{ */

#include <functional>

//}

namespace mrs_uav_status::status
{

// Lets UavStatusCore read the node clock without linking against any ROS client-library type
// itself -- injected by ros_status, same reason as CommandSink but kept separate from it since
// it's a data source, not an outbound action. Used where the tick-start freshness snapshot is
// too stale -- e.g. stamping renderServiceResult()'s clear-time after a blocking service call
// returns.
struct Clock
{
  // Wall-clock "now" in seconds.
  std::function<double()> now;
};

} // namespace mrs_uav_status::status

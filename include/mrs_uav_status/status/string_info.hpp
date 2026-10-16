#pragma once

/* includes //{ */

#include <string>

//}

namespace mrs_uav_status::status
{

// One display_string entry published by a user node, with its id (for dedup) and last-seen time
// (for expiry). The timestamp is plain seconds from the node clock -- status/ carries no ROS types.
struct StringInfo
{
  std::string id;
  std::string display_string;
  bool        persistent = false;
  double      last_time  = 0.0;

  /* StringInfo() //{ */

  StringInfo(double last_time_in, const std::string &display_string_in, const std::string &id_in, bool persistent_in)
      : id(id_in), display_string(display_string_in), persistent(persistent_in), last_time(last_time_in) {
  }

  //}
};

} // namespace mrs_uav_status::status

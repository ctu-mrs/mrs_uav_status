#pragma once

/* includes //{ */

#include <cstdint>
#include <exception>
#include <initializer_list>
#include <string>
#include <vector>

//}

namespace mrs_uav_status::utils
{

// Mirrors mrs_msgs::msg::GeneralRobotInfo::ROBOT_TYPE_* -- named locally so utils/ keeps no
// dependency on ROS message types.
inline constexpr uint8_t ROBOT_TYPE_DRONE        = 0;
inline constexpr uint8_t ROBOT_TYPE_BOAT         = 1;
inline constexpr uint8_t ROBOT_TYPE_GROUND_ROBOT = 2;

/* splitByChar() //{ */

// Splits input on delimiter (returns {""} for an empty input).
inline std::vector<std::string> splitByChar(const std::string &input, char delimiter) {
  if (input.empty()) {
    return {""};
  }

  std::vector<std::string> result;
  std::size_t              start = 0;

  while (start < input.size()) {
    const std::size_t pos = input.find(delimiter, start);
    if (pos == std::string::npos) {
      result.emplace_back(input.substr(start));
      break;
    }
    result.emplace_back(input.substr(start, pos - start));
    start = pos + 1;
  }

  return result;
}

//}

/* withActiveFirst() //{ */

// Returns {active, available\active} — the legacy UavStatus convention is
// "first element is the active one, rest are alternates."
inline std::vector<std::string> withActiveFirst(const std::string &active, const std::vector<std::string> &available) {
  std::vector<std::string> out;
  out.reserve(available.size() + 1);
  if (!active.empty()) {
    out.push_back(active);
  }
  for (const auto &s : available) {
    if (s != active) {
      out.push_back(s);
    }
  }
  return out;
}

//}

/* robotTypeToString() //{ */

// No UNKNOWN sentinel in the enum (0 == DRONE) — caller must check message freshness separately.
inline std::string robotTypeToString(uint8_t robot_type) {
  switch (robot_type) {
  case ROBOT_TYPE_DRONE:
    return "DRONE";
  case ROBOT_TYPE_BOAT:
    return "BOAT";
  case ROBOT_TYPE_GROUND_ROBOT:
    return "UGV";
  default:
    return "UNKNOWN";
  }
}

//}

/* isFlightStateUnhealthy() //{ */

// DiagnosticsManager's UavInfo.flight_state: true when it should be highlighted -- no link, a pilot flying (MANUAL),
// an emergency (EHOVER, ELAND, FAILSAFE), UNKNOWN, or anything not recognised; normal ground and MRS flight states are not
inline bool isFlightStateUnhealthy(const std::string &flight_state) {
  for (const char *healthy : {"DISARMED", "ARMED", "OFFBOARD", "TAKEOFF", "HOVER", "GOTO", "TRAJECTORY", "LAND", "MIDAIR", "RC_MODE"}) {
    if (flight_state == healthy) {
      return false;
    }
  }
  return true;
}

//}

/* isFlyingAutonomously() //{ */

// DiagnosticsManager's UavInfo.flight_state: true while the MRS system is in command in the air, emergencies and midair
// activation included (mirrors mrs_uav_managers' is_flying_autonomously()); false on the ground, for a pilot flying
// (MANUAL), and for anything not recognised
inline bool isFlyingAutonomously(const std::string &flight_state) {
  for (const char *flying : {"TAKEOFF", "LAND", "HOVER", "GOTO", "TRAJECTORY", "MIDAIR", "RC_MODE", "EHOVER", "ELAND", "FAILSAFE"}) {
    if (flight_state == flying) {
      return true;
    }
  }
  return false;
}

//}

/* isOnGround() //{ */

// DiagnosticsManager's UavInfo.flight_state: true for the ground states, from which a takeoff can be started
inline bool isOnGround(const std::string &flight_state) {
  return flight_state == "DISARMED" || flight_state == "ARMED" || flight_state == "OFFBOARD";
}

//}

/* splitNotResponding() //{ */

struct SplitErrors
{
  std::vector<std::string> specific;
  std::vector<std::string> not_responding;
};

// Matches the "<node>.<component>: not responding" format from DiagnosticsManager's find_error_roots() loop.
inline SplitErrors splitNotResponding(const std::vector<std::string> &errors) {
  const std::string suffix      = ": not responding";
  const std::string main_suffix = ".main";

  SplitErrors out;
  for (const auto &e : errors) {
    if (e != suffix && e.ends_with(suffix)) {
      std::string source = e.substr(0, e.size() - suffix.size());
      if (source != main_suffix && source.ends_with(main_suffix)) {
        source.resize(source.size() - main_suffix.size());
      }
      out.not_responding.push_back(source);
    } else {
      out.specific.push_back(e);
    }
  }
  return out;
}

//}

/* findSensor() //{ */

// Find the first SensorStatus of a given type. Returns nullptr if not present.
// Templated so this works for status::SensorStatusData without a dependency on status/ or mrs_msgs.
template <typename SensorT>
inline const SensorT *findSensor(const std::vector<SensorT> &sensors, uint8_t type) {
  for (const auto &s : sensors) {
    if (s.type == type) {
      return &s;
    }
  }
  return nullptr;
}

//}

/* lookupDetail() //{ */

// Look up a value in a KeyValue list. Returns fallback if the key isn't present.
// Templated for the same reason as findSensor() above -- works for both
// diagnostic_msgs::msg::KeyValue and status::KeyValueData.
template <typename KeyValueT>
inline std::string lookupDetail(const std::vector<KeyValueT> &details, const std::string &key, const std::string &fallback = "") {
  for (const auto &kv : details) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return fallback;
}

//}

/* parseDoubleOr() //{ */

// Parses s as a double; returns fallback if empty or unparseable.
inline double parseDoubleOr(const std::string &s, double fallback) {
  if (s.empty()) {
    return fallback;
  }
  try {
    return std::stod(s);
  }
  catch (const std::exception &) {
    return fallback;
  }
}

//}

/* parseLongOr() //{ */

// Parses s as a long; returns fallback if empty or unparseable.
inline long parseLongOr(const std::string &s, long fallback) {
  if (s.empty()) {
    return fallback;
  }
  try {
    return std::stol(s);
  }
  catch (const std::exception &) {
    return fallback;
  }
}

//}

} // namespace mrs_uav_status::utils

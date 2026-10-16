#pragma once

/* includes //{ */

#include <cstdint>
#include <string>
#include <vector>

//}

namespace mrs_uav_status::status
{

// Mirrors mrs_msgs::msg::CpuLoad. Both fields are used.
struct CpuLoadData
{
  std::string node_name;
  float       cpu_load = -1.0f;
};

// Mirrors mrs_msgs::msg::SensorStatus, trimmed: `ready`, `topic` are never read by this package.
// `details` is retained (as KeyValueData below) for the gnss/magnetometer detail lookups via
// utils::lookupDetail(). `type`/`level` reuse the ROS enums' numeric values
// (mrs_msgs::msg::SensorStatus::{OK,WARN,ERROR,STALE}, ::TYPE_*) so existing comparisons work unchanged.
inline constexpr uint8_t SENSOR_STATUS_OK    = 0;
inline constexpr uint8_t SENSOR_STATUS_WARN  = 1;
inline constexpr uint8_t SENSOR_STATUS_ERROR = 2;
inline constexpr uint8_t SENSOR_STATUS_STALE = 3;

// Subset of mrs_msgs::msg::SensorStatus's `type` constants actually read by this package
// (utils::findSensor() call sites).
inline constexpr uint8_t SENSOR_TYPE_AUTOPILOT    = 0;
inline constexpr uint8_t SENSOR_TYPE_GNSS         = 2;
inline constexpr uint8_t SENSOR_TYPE_MAGNETOMETER = 5;

// Mirrors diagnostic_msgs::msg::KeyValue (the element type of SensorStatus::details).
struct KeyValueData
{
  std::string key;
  std::string value;
};

struct SensorStatusData
{
  uint8_t                   type = 9; // TYPE_UNKNOWN
  std::string               name;
  float                     rate  = -1.0f;
  uint8_t                   level = SENSOR_STATUS_OK;
  std::string               message;
  std::vector<KeyValueData> details;
};

// Mirrors mrs_msgs::msg::OnboardComputerInfo, trimmed: `cpu_temperature`, `wifi_interface`,
// `wifi_signal_dbm`, `wifi_link_quality` are never read.
struct OnboardComputerInfoData
{
  float                    cpu_load  = -1.0f;
  float                    cpu_ghz   = -1.0f;
  float                    free_ram  = -1.0f;
  float                    total_ram = -1.0f;
  int32_t                  free_hdd  = -1;
  std::vector<CpuLoadData> node_cpu_loads;
};

// Mirrors mrs_msgs::msg::BatteryState, trimmed: `percentage` is never read.
struct BatteryStateData
{
  float voltage    = -1.0f;
  float current    = -1.0f;
  float wh_drained = -1.0f;
};

// Mirrors mrs_msgs::msg::GeneralRobotInfo, trimmed: `robot_ip_address` and
// `battery_state.percentage` are never read.
struct GeneralRobotInfoData
{
  std::string              robot_name;
  uint8_t                  robot_type                  = 0; // ROBOT_TYPE_DRONE
  bool                     ready_for_automatic_takeoff = false;
  std::vector<std::string> missing_for_automatic_takeoff;
  std::vector<std::string> errors;
  BatteryStateData         battery_state;
};

// Mirrors mrs_msgs::msg::SystemHealthInfo. Fully used at this level; nested types are trimmed
// (see OnboardComputerInfoData, SensorStatusData).
struct SystemHealthInfoData
{
  OnboardComputerInfoData       onboard_computer_info;
  float                         hw_api_rate           = -1.0f;
  float                         control_manager_rate  = -1.0f;
  float                         state_estimation_rate = -1.0f;
  std::vector<SensorStatusData> available_sensors;
};

// Mirrors mrs_msgs::msg::UavInfo. Fully used, no trimming.
struct UavInfoData
{
  std::string flight_state    = "unknown";
  float       flight_duration = -1.0f;
  bool        armed           = false;
  bool        offboard        = false;
  float       mass_nominal    = -1.0f;
  float       mass_estimate   = -1.0f;
};

// Mirrors mrs_msgs::msg::ControlInfo. Fully used, no trimming. cmd_pose (mrs_msgs::Reference) is
// flattened to 4 doubles since only position.{x,y,z} and heading are read.
struct ControlInfoData
{
  std::string              active_controller = "unknown";
  std::vector<std::string> available_controllers;
  std::string              active_gains = "unknown";
  std::vector<std::string> available_gains;
  std::string              active_tracker = "unknown";
  std::vector<std::string> available_trackers;
  std::string              active_constraints = "unknown";
  std::vector<std::string> available_constraints;
  float                    throttle   = -1.0f;
  double                   cmd_pose_x = 0.0, cmd_pose_y = 0.0, cmd_pose_z = 0.0, cmd_pose_heading = 0.0;
  bool                     flying_normally     = false;
  bool                     have_goal           = false;
  bool                     tracking_trajectory = false;
  bool                     callbacks_enabled   = false;
};

// Mirrors mrs_msgs::msg::CollisionAvoidanceInfo, trimmed: `other_robots_visible` is only ever
// read via .size(), so only the count is kept.
struct CollisionAvoidanceInfoData
{
  bool        collision_avoidance_enabled = false;
  bool        avoiding_collision          = false;
  bool        bumper_active               = false;
  std::size_t num_other_robots_visible    = 0;
};

// Mirrors mrs_msgs::msg::StateEstimationInfo, trimmed: `global_pose`, `above_ground_level_height`,
// `velocity`, `acceleration`, `running_estimators` are never read. local_pose (mrs_msgs::Reference)
// is flattened to 4 doubles.
struct StateEstimationInfoData
{
  std::string              frame_id;
  double                   pos_x = 0.0, pos_y = 0.0, pos_z = 0.0, heading = 0.0;
  std::string              current_estimator;
  std::vector<std::string> switchable_estimators;
  std::string              horizontal_estimator = "unknown";
  std::string              vertical_estimator   = "unknown";
  std::string              heading_estimator    = "unknown";
  std::string              agl_estimator        = "unknown";
  float                    max_flight_z         = -1.0f;
};

// Mirrors mrs_msgs::msg::State::STATE_RC_MODE, named here since StateData is trimmed to just
// the raw `state` field with no room for the message's own enum members.
inline constexpr uint8_t STATE_RC_MODE = 7;

// Mirrors mrs_msgs::msg::State, trimmed to the one field read (rc-mode check against
// STATE_RC_MODE above).
struct StateData
{
  uint8_t state = 0; // STATE_UNKNOWN
};

// Derived, not pushed directly from a message: the 4 fields used for printBox() border
// coloring, computed once per render tick from GeneralRobotInfoData/CollisionAvoidanceInfoData/
// ControlInfoData.
struct BorderStatus
{
  bool avoiding_collision = false;
  bool bumper_active      = false;
  bool can_takeoff        = false;
  bool null_tracker       = false;
};

// Per-topic "has this ever arrived and not gone stale" flags, computed by UavStatus and
// pushed once per render tick.
struct Freshness
{
  bool general_robot_info       = false;
  bool collision_avoidance_info = false;
  bool uav_info                 = false;
  bool system_health_info       = false;
  bool state_estimation_info    = false;
  bool control_info             = false;
  bool uav_state                = false;
};

// One render tick's complete view of the world, deep-copied out of UavStatusCore under its mutex
// and handed to NcursesTui::setSnapshot(). NcursesTui reads this (render thread only, no locking) instead of
// keeping its own message snapshots.
struct RenderSnapshot
{
  double                     now_seconds = 0.0;
  Freshness                  freshness;
  BorderStatus               border_status;
  GeneralRobotInfoData       general_robot_info;
  StateEstimationInfoData    state_estimation_info;
  ControlInfoData            control_info;
  CollisionAvoidanceInfoData collision_avoidance_info;
  UavInfoData                uav_info;
  SystemHealthInfoData       system_health_info;
  StateData                  uav_state;

  // Live display_string entries (already pruned), in insertion order.
  std::vector<std::string> display_strings;
};

} // namespace mrs_uav_status::status

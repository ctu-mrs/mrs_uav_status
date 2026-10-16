/* includes //{ */

#include <mrs_uav_status/uav_status.hpp>
#include <mrs_uav_status/utils/helpers.hpp>

//}

// Anonymous namespace: gives toData() internal linkage, since it's only ever called from this
// file's callbacks below.
namespace
{

/* toData() //{ */

// Converts each subscribed mrs_msgs type to its plain status::*Data mirror (see data_types.hpp),
// trimming unused fields along the way.

mrs_uav_status::status::GeneralRobotInfoData toData(const mrs_msgs::msg::GeneralRobotInfo &msg) {
  return {
      .robot_name                    = msg.robot_name,
      .robot_type                    = msg.robot_type,
      .ready_for_automatic_takeoff   = msg.ready_for_automatic_takeoff,
      .missing_for_automatic_takeoff = msg.missing_for_automatic_takeoff,
      .errors                        = msg.errors,
      .battery_state                 = {.voltage = msg.battery_state.voltage, .current = msg.battery_state.current, .wh_drained = msg.battery_state.wh_drained},
  };
}

mrs_uav_status::status::StateEstimationInfoData toData(const mrs_msgs::msg::StateEstimationInfo &msg) {
  return {
      .frame_id              = msg.header.frame_id,
      .pos_x                 = msg.local_pose.position.x,
      .pos_y                 = msg.local_pose.position.y,
      .pos_z                 = msg.local_pose.position.z,
      .heading               = msg.local_pose.heading,
      .current_estimator     = msg.current_estimator,
      .switchable_estimators = msg.switchable_estimators,
      .horizontal_estimator  = msg.horizontal_estimator,
      .vertical_estimator    = msg.vertical_estimator,
      .heading_estimator     = msg.heading_estimator,
      .agl_estimator         = msg.agl_estimator,
      .max_flight_z          = msg.max_flight_z,
  };
}

mrs_uav_status::status::ControlInfoData toData(const mrs_msgs::msg::ControlInfo &msg) {
  return {
      .active_controller     = msg.active_controller,
      .available_controllers = msg.available_controllers,
      .active_gains          = msg.active_gains,
      .available_gains       = msg.available_gains,
      .active_tracker        = msg.active_tracker,
      .available_trackers    = msg.available_trackers,
      .active_constraints    = msg.active_constraints,
      .available_constraints = msg.available_constraints,
      .throttle              = msg.throttle,
      .cmd_pose_x            = msg.cmd_pose.position.x,
      .cmd_pose_y            = msg.cmd_pose.position.y,
      .cmd_pose_z            = msg.cmd_pose.position.z,
      .cmd_pose_heading      = msg.cmd_pose.heading,
      .flying_normally       = msg.flying_normally,
      .have_goal             = msg.have_goal,
      .tracking_trajectory   = msg.tracking_trajectory,
      .callbacks_enabled     = msg.callbacks_enabled,
  };
}

mrs_uav_status::status::CollisionAvoidanceInfoData toData(const mrs_msgs::msg::CollisionAvoidanceInfo &msg) {
  return {
      .collision_avoidance_enabled = msg.collision_avoidance_enabled,
      .avoiding_collision          = msg.avoiding_collision,
      .bumper_active               = msg.bumper_active,
      .num_other_robots_visible    = msg.other_robots_visible.size(),
  };
}

mrs_uav_status::status::UavInfoData toData(const mrs_msgs::msg::UavInfo &msg) {
  return {
      .flight_state    = msg.flight_state,
      .flight_duration = msg.flight_duration,
      .armed           = msg.armed,
      .offboard        = msg.offboard,
      .mass_nominal    = msg.mass_nominal,
      .mass_estimate   = msg.mass_estimate,
  };
}

mrs_uav_status::status::SystemHealthInfoData toData(const mrs_msgs::msg::SystemHealthInfo &msg) {
  mrs_uav_status::status::SystemHealthInfoData data;
  data.onboard_computer_info.cpu_load  = msg.onboard_computer_info.cpu_load;
  data.onboard_computer_info.cpu_ghz   = msg.onboard_computer_info.cpu_ghz;
  data.onboard_computer_info.free_ram  = msg.onboard_computer_info.free_ram;
  data.onboard_computer_info.total_ram = msg.onboard_computer_info.total_ram;
  data.onboard_computer_info.free_hdd  = msg.onboard_computer_info.free_hdd;
  for (const auto &n : msg.onboard_computer_info.node_cpu_loads) {
    data.onboard_computer_info.node_cpu_loads.push_back({.node_name = n.node_name, .cpu_load = n.cpu_load});
  }
  data.hw_api_rate           = msg.hw_api_rate;
  data.control_manager_rate  = msg.control_manager_rate;
  data.state_estimation_rate = msg.state_estimation_rate;
  for (const auto &s : msg.available_sensors) {
    mrs_uav_status::status::SensorStatusData sd;
    sd.type    = s.type;
    sd.name    = s.name;
    sd.rate    = s.rate;
    sd.level   = s.level;
    sd.message = s.message;
    for (const auto &d : s.details) {
      sd.details.push_back({.key = d.key, .value = d.value});
    }
    data.available_sensors.push_back(std::move(sd));
  }
  return data;
}

mrs_uav_status::status::StateData toData(const mrs_msgs::msg::State &msg) {
  return {.state = msg.state};
}

//}

} // namespace

namespace mrs_uav_status
{

/* UavStatus() //{ */

UavStatus::UavStatus() : Node("mrs_status_menu") {

  tui::NcursesTui::initTerminal();

  initialize();
}

//}

/* ~UavStatus() //{ */

UavStatus::~UavStatus() {
  timer_render_.reset();
  tui::NcursesTui::shutdownTerminal();
}

//}

/* initialize() //{ */

void UavStatus::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | ------------------------- Parameters ------------------------- |

  /*//{ load parameters */

  mrs_lib::ParamLoader param_loader(node_);

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config_public");

  std::string                 pwd, colorscheme, turbo_remote_constraints, uav_name;
  double                      update_rate, update_rate_slow, resize_rate;
  Eigen::Matrix<double, 4, 1> goto_values_mat;
  bool                        colorblind_mode = false;
  bool                        start_minimized = false;

  param_loader.loadParam("pwd", pwd);
  param_loader.loadParam("colorscheme", colorscheme);
  param_loader.loadParam("uav_name", uav_name);
  param_loader.loadParam("mrs_uav_status/rates/update_rate", update_rate);
  param_loader.loadParam("mrs_uav_status/rates/update_rate_slow", update_rate_slow);
  param_loader.loadParam("mrs_uav_status/rates/resize_rate", resize_rate);
  param_loader.loadParam("mrs_uav_status/data_timeout", data_timeout_s_);
  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", turbo_remote_constraints);
  param_loader.loadParam("mrs_uav_status/display/colorblind_mode", colorblind_mode);
  param_loader.loadParam("mrs_uav_status/enable_profiler", _profiler_enabled_);
  param_loader.loadParam("mrs_uav_status/display/start_minimized", start_minimized);
  std::vector<std::string> service_list;
  param_loader.loadParam("mrs_uav_status/menu/service_list", service_list);
  param_loader.loadMatrixStatic("mrs_uav_status/menu/goto_values", goto_values_mat);

  /*//}*/

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }
  RCLCPP_INFO(node_->get_logger(), "All params loaded!");

  // | --------------------- NcursesTui & UavStatusCore --------------------- |

  const std::string         display_config_filename = pwd + "/.mrs_status_display_config~";
  const std::vector<double> goto_values(goto_values_mat.data(), goto_values_mat.data() + goto_values_mat.size());

  const tui::NcursesTui::Params tui_params{
      .colorscheme             = colorscheme,
      .colorblind_mode         = colorblind_mode,
      .start_minimized         = start_minimized,
      .display_config_filename = display_config_filename,
  };

  const status::UavStatusCore::Params core_params{
      .turbo_remote_constraints = turbo_remote_constraints,
      .goto_values              = goto_values,
  };

  status::CommandSink command_sink = buildCommandSink(service_list, uav_name);
  status::Clock       core_clock{.now = [this]() { return clock_->now().seconds(); }};

  tui_ = std::make_unique<tui::NcursesTui>(tui_params);
  tui_->updateTermSize();
  tui_->setupWindows();
  tui_->loadDisplayConfig();

  core_ = std::make_unique<status::UavStatusCore>(std::move(command_sink), core_params, core_clock);

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;
  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  timer_render_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(update_rate, clock_), std::bind(&UavStatus::timerRender, this));

  slow_period_       = rclcpp::Duration::from_seconds(1.0 / update_rate_slow);
  resize_period_     = rclcpp::Duration::from_seconds(1.0 / resize_rate);
  last_slow_run_     = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_resize_check_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | ------------------------ Subscribers ------------------------ |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_general_robot_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>(shopts, "~/general_robot_info_in", &UavStatus::callbackGeneralRobotInfo, this);
  sh_state_estimation_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>(shopts, "~/state_estimation_info_in", &UavStatus::callbackStateEstimationInfo, this);
  sh_control_info_             = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>(shopts, "~/control_info_in", &UavStatus::callbackControlInfo, this);
  sh_collision_avoidance_info_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo>(shopts, "~/collision_avoidance_info_in",
                                                                                                   &UavStatus::callbackCollisionAvoidanceInfo, this);
  sh_uav_info_                 = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>(shopts, "~/uav_info_in", &UavStatus::callbackUavInfo, this);
  sh_system_health_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>(shopts, "~/system_health_info_in", &UavStatus::callbackSystemHealthInfo, this);
  sh_uav_state_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::State>(shopts, "~/uav_state_in", &UavStatus::callbackUavState, this);
  // Custom-display string topic — anything published here lands in the NcursesTui's
  // Strings window. Supports "-id <key> -p <space-separated text>" preamble.
  sh_display_string_ = mrs_lib::SubscriberHandler<std_msgs::msg::String>(shopts, "~/display_string_in", &UavStatus::callbackDisplayString, this);

  // | --------------------- Finish the init --------------------- |

  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* buildCommandSink() //{ */

status::CommandSink UavStatus::buildCommandSink(const std::vector<std::string> &service_list, const std::string &uav_name) {

  sc_goto_reference_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/goto_reference_out", cbkgrp_sc_);
  sc_velocity_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv>(node_, "~/velocity_reference_out", cbkgrp_sc_);
  sc_set_constraints_    = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_constraints_out", cbkgrp_sc_);
  sc_set_gains_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_gains_out", cbkgrp_sc_);
  sc_set_controller_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_controller_out", cbkgrp_sc_);
  sc_set_tracker_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_tracker_out", cbkgrp_sc_);
  sc_set_estimator_      = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_estimator_out", cbkgrp_sc_);
  sc_hover_              = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out", cbkgrp_sc_);

  status::CommandSink command_sink;

  command_sink.sendGoto = [this](double x, double y, double z, double heading, const std::string &frame_id) -> status::CommandSink::ServiceResult {
    auto request                  = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request->reference.position.x = x;
    request->reference.position.y = y;
    request->reference.position.z = z;
    request->reference.heading    = heading;
    request->header.frame_id      = frame_id;
    auto response                 = sc_goto_reference_.callSync(request);
    if (!response) {
      return {false, "service could not be called"};
    }
    return {response.value()->success, response.value()->message};
  };

  command_sink.sendVelocityReference = [this](double vx, double vy, double vz, double heading_rate, const std::string &frame_id) {
    auto request                                  = std::make_shared<mrs_msgs::srv::VelocityReferenceStampedSrv::Request>();
    request->reference.reference.velocity.x       = vx;
    request->reference.reference.velocity.y       = vy;
    request->reference.reference.velocity.z       = vz;
    request->reference.reference.heading_rate     = heading_rate;
    request->reference.reference.use_heading_rate = true;
    request->reference.header.frame_id            = frame_id;
    request->reference.header.stamp               = clock_->now();
    sc_velocity_reference_.callSync(request);
  };

  auto makeStringSetter = [this](mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &client) {
    return [this, &client](const std::string &value) -> status::CommandSink::ServiceResult {
      auto request   = std::make_shared<mrs_msgs::srv::String::Request>();
      request->value = value;
      auto response  = client.callSync(request);
      if (!response) {
        return {false, "service could not be called"};
      }
      return {response.value()->success, response.value()->message};
    };
  };
  command_sink.setConstraints = makeStringSetter(sc_set_constraints_);
  command_sink.setGains       = makeStringSetter(sc_set_gains_);
  command_sink.setController  = makeStringSetter(sc_set_controller_);
  command_sink.setTracker     = makeStringSetter(sc_set_tracker_);
  command_sink.setEstimator   = makeStringSetter(sc_set_estimator_);

  command_sink.hover = [this]() -> status::CommandSink::ServiceResult {
    auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = sc_hover_.callSync(request);
    if (!response) {
      return {false, "service could not be called"};
    }
    return {response.value()->success, response.value()->message};
  };

  sc_extra_services_.reserve(service_list.size());
  for (const auto &service_input : service_list) {
    std::vector<std::string> results = utils::splitByChar(service_input, ' ');

    if (results.size() < 2 || results[0].empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid service entry: '%s'. Each entry must contain at least a service name and a display name, separated by a space.",
                   service_input.c_str());
      continue;
    }

    for (unsigned long j = 2; j < results.size(); j++) {
      results[1] = results[1] + " " + results[j];
    }

    const std::string service_name = (results[0].at(0) == '/') ? results[0] : "/" + uav_name + "/" + results[0];
    const std::string display_name = results[1];

    sc_extra_services_.emplace_back(node_, service_name, cbkgrp_sc_);
    const std::size_t idx = sc_extra_services_.size() - 1;

    command_sink.extra_services.push_back({display_name, [this, idx]() -> status::CommandSink::ServiceResult {
                                             auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
                                             auto response = sc_extra_services_[idx].callSync(request);
                                             if (!response) {
                                               return {false, "service could not be called"};
                                             }
                                             return {response.value()->success, response.value()->message};
                                           }});
  }

  return command_sink;
}

//}

/* timerRender() //{ */

void UavStatus::timerRender() {

  if (!is_initialized_) {
    return;
  }

  const rclcpp::Time now         = clock_->now();
  const double       now_seconds = now.seconds();
  bool               resized     = false;

  // hasMsg() guards against sim-time-near-zero at boot; the elapsed-time check catches mid-flight stalls.
  auto                    is_fresh = [&now, this](const auto &sh) { return sh.hasMsg() && (now - sh.lastMsgTime()).seconds() < data_timeout_s_; };
  const status::Freshness freshness{
      .general_robot_info       = is_fresh(sh_general_robot_info_),
      .collision_avoidance_info = is_fresh(sh_collision_avoidance_info_),
      .uav_info                 = is_fresh(sh_uav_info_),
      .system_health_info       = is_fresh(sh_system_health_info_),
      .state_estimation_info    = is_fresh(sh_state_estimation_info_),
      .control_info             = is_fresh(sh_control_info_),
      .uav_state                = is_fresh(sh_uav_state_),
  };

  core_->setFreshness(freshness);
  tui_->setSnapshot(core_->snapshot(now_seconds));

  // Resize before anything else draws; force a slow redraw right after so it isn't left blank.
  if (now - last_resize_check_ >= resize_period_) {
    last_resize_check_                = now;
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("resize");
    resized                           = tui_->resize();
  }
  if (resized || now - last_slow_run_ >= slow_period_) {
    last_slow_run_                    = now;
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("renderSlow");
    tui_->renderSlow();
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("renderFast");
    tui_->renderFast();
  }

  tui_->blankBottomWindow();

  const int key = tui_->pollKey();
  core_->tick(now_seconds, key, *tui_);

  tui_->refreshTopBar();
  tui_->commitFrame();
}

//}

/* callbacks //{ */

void UavStatus::callbackGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onGeneralRobotInfo(toData(*msg));
}

void UavStatus::callbackStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onStateEstimationInfo(toData(*msg));
}

void UavStatus::callbackControlInfo(const mrs_msgs::msg::ControlInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onControlInfo(toData(*msg));
}

void UavStatus::callbackCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onCollisionAvoidanceInfo(toData(*msg));
}

void UavStatus::callbackUavInfo(const mrs_msgs::msg::UavInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onUavInfo(toData(*msg));
}

void UavStatus::callbackSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onSystemHealthInfo(toData(*msg));
}

void UavStatus::callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onUavState(toData(*msg));
}

void UavStatus::callbackDisplayString(const std_msgs::msg::String::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  core_->onString(clock_->now().seconds(), msg->data);
}

//}

} // namespace mrs_uav_status

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mrs_uav_status::UavStatus>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

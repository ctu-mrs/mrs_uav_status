#pragma once

/* includes //{ */

#include <memory>
#include <atomic>

#include <mrs_uav_status/tui/ncurses_tui.hpp>
#include <mrs_uav_status/status/uav_status_core.hpp>

#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/velocity_reference_stamped_srv.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>

//}

#if USE_ROS_TIMER == 1
using TimerType = mrs_lib::ROSTimer;
#else
using TimerType = mrs_lib::ThreadTimer;
#endif

namespace mrs_uav_status
{

// The package's only ROS2 node: owns every rclcpp handle (subscribers, service clients, the render
// timer), converts incoming messages to plain status::*Data, and converts rclcpp::Time to a plain
// double before handing anything to the ROS-free layers below.
class UavStatus : public mrs_lib::Node {

public:
  // Calls tui::NcursesTui::initTerminal(), then initialize().
  UavStatus();
  // Stops the render timer, then calls tui::NcursesTui::shutdownTerminal().
  ~UavStatus();

private:
  // Loads params, constructs the NcursesTui, starts the render timer, and subscribes to all topics.
  void initialize();
  // Constructs the sc_*_ service clients and builds the status::CommandSink that initialize()
  // hands off to the UavStatusCore ctor.
  status::CommandSink buildCommandSink(const std::vector<std::string> &service_list, const std::string &uav_name);

  bool _profiler_enabled_ = false;

  rclcpp::Node::SharedPtr          node_;
  rclcpp::Clock::SharedPtr         clock_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>       sh_general_robot_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>    sh_state_estimation_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>            sh_control_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo> sh_collision_avoidance_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>                sh_uav_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>       sh_system_health_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::State>                  sh_uav_state_;
  mrs_lib::SubscriberHandler<std_msgs::msg::String>                 sh_display_string_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>         sc_goto_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv> sc_velocity_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_constraints_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_estimator_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_hover_;
  std::vector<mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>>        sc_extra_services_;

  std::shared_ptr<TimerType> timer_render_;

  rclcpp::Duration slow_period_{0, 0};
  rclcpp::Duration resize_period_{0, 0};
  rclcpp::Time     last_slow_run_;
  rclcpp::Time     last_resize_check_;
  double           data_timeout_s_ = 0.0; // Seconds without a message before data is considered stale.

  // Per-tick entry point: updates freshness/resize/render, reads one key, and routes it through
  // core_'s STANDARD/REMOTE/MAIN_MENU/GOTO_MENU/DISPLAY_MENU state machine.
  void timerRender();
  // Forwards the message to the matching status::UavStatusCore::on*() setter.
  void callbackGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr msg);
  void callbackStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr msg);
  void callbackControlInfo(const mrs_msgs::msg::ControlInfo::ConstSharedPtr msg);
  void callbackCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr msg);
  void callbackUavInfo(const mrs_msgs::msg::UavInfo::ConstSharedPtr msg);
  void callbackSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr msg);
  void callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg);
  void callbackDisplayString(const std_msgs::msg::String::ConstSharedPtr msg);

  std::atomic<bool> is_initialized_ = false;

  mrs_lib::Profiler                      profiler_;
  std::unique_ptr<tui::NcursesTui>       tui_;
  std::unique_ptr<status::UavStatusCore> core_;
};

} // namespace mrs_uav_status

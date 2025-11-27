/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/node_cpu_load.hpp>
#include <commons.h>

#include <iostream>
#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

/* #include <ros/xmlrpc_manager.h> */
/* #include <XmlRpcClient.h> */

#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/mpc_tracker_diagnostics.hpp>
#include <mrs_msgs/msg/custom_topic.hpp>
#include <mrs_msgs/msg/controller_diagnostics.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/custom_topic.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/gps_info.hpp>

#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_msgs/srv/string.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/profiler.h>

#include <cmath>

using namespace std;

//}

/* defines //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

/* class Acquisition //{ */

namespace mrs_uav_status
{

class Acquisition : public mrs_lib::Node {

public:
  Acquisition(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  void initialize();

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  std::mutex mutex_status_msg_;

  mrs_msgs::msg::UavStatus      uav_status_;
  mrs_msgs::msg::UavStatusShort uav_status_short_;

  // | ------------------------- Timers ------------------------- |

  std::shared_ptr<TimerType> timer_status_;

  std::shared_ptr<TimerType> timer_host_info_;

  void timerStatus();

  void timerHostInfo();

  // | ------------------------ Utility ----------------------- |

  int getPort(std::string uri);

  // | ------------------------ Callbacks ----------------------- |

  void callbackUavState(const mrs_msgs::msg::UavState::ConstSharedPtr msg);
  void callbackTrackerCommand(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);
  void callbackEstimationDiaag(const mrs_msgs::msg::EstimationDiagnostics::ConstSharedPtr msg);
  void callbackMpcTrackerDiag(const mrs_msgs::msg::MpcTrackerDiagnostics::ConstSharedPtr msg);
  void callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg);
  void callbackBatteryState(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  void callbackThrottle(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void callbackMassEstimate(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void callbackNominalMass(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);
  void callbackGainManagerDiag(const mrs_msgs::msg::GainManagerDiagnostics::ConstSharedPtr msg);
  void callbackConstraintManagerDiagnostics(const mrs_msgs::msg::ConstraintManagerDiagnostics::ConstSharedPtr msg);
  void callbackTfStatic(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);
  void callbackHwApiGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void callbackHwApiGNSSStatus(const mrs_msgs::msg::GpsInfo::ConstSharedPtr msg);
  void callbackHwApiOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackAutostartReady(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void callbackMagnetometer(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
  void callbackString(const std_msgs::msg::String::ConstSharedPtr msg);

  // generic callback, for any topic, to monitor its rate
  void callbackGeneric(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string topic, const int id);

  // | ------------------------- Windows ------------------------ |

  void uavStateHandler();
  void nodeCpuLoadHandler();
  void updateNodeList();
  void controlManagerHandler();
  void hwApiDiagHandler();
  void genericTopicHandler();
  void stringHandler();

  int BUFFER_LEN_SECS = 2;

  double uav_state_expected_rate_          = 100.0;
  double control_manager_expected_rate_    = 10.0;
  double hw_api_odometry_expected_rate_    = 100.0;
  double hw_api_gnss_expected_rate_        = 100.0;
  double hw_api_gnss_status_expected_rate_ = 1.0;
  double hw_api_state_expected_rate_       = 100.0;
  double hw_api_battery_expected_rate_     = 0.5;
  double hw_api_mag_expected_rate_         = 10;

  TopicInfo uav_state_ts_;
  TopicInfo control_manager_ts_;
  TopicInfo hw_api_odometry_ts_;
  TopicInfo hw_api_gnss_ts_;
  TopicInfo hw_api_gnss_status_ts_;
  TopicInfo hw_api_state_ts_;
  TopicInfo hw_api_battery_ts_;
  TopicInfo hw_api_mag_ts_;

  double general_info_window_rate_  = 1;
  double generic_topic_window_rate_ = 1;

  // General info window

  void getCpuLoad();
  void getCpuTemperature();
  void getMemLoad();
  void getCpuFreq();
  void getDiskSpace();

  long last_idle_  = 0;
  long last_total_ = 0;

  long total_diff_      = 0;
  long last_total_diff_ = 0;

  int cpu_cores_ = 1;

  // | ---------------------- Misc routines --------------------- |

  void prefillUavStatus();

  void flightTimeHandler();

  void setupGenericCallbacks();

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ----------------------- Subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>                     sh_uav_state_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>               sh_tracker_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>        sh_estimator_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::MpcTrackerDiagnostics>        sh_mpc_tracker_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>                  sh_hw_api_status_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>                 sh_hw_api_gnss_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>                      sh_hw_api_gnss_status_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>                     sh_hw_api_odom_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                      sh_mass_estimate_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                      sh_nominal_mass_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                      sh_throttle_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>              sh_battery_state_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>       sh_gain_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics> sh_constraint_manager_diag_;
  mrs_lib::SubscriberHandler<std_msgs::msg::String>                       sh_string_;
  mrs_lib::SubscriberHandler<tf2_msgs::msg::TFMessage>                    sh_tf_static_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Bool>                         sh_autostart_ready_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>             sh_magnetometer_;

  // | ----------------------- Publishers ---------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::UavStatus>      ph_uav_status_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::UavStatusShort> ph_uav_status_short_;

  // | -------------------- UAV configuration ------------------- |

  string _uav_name_;
  string _uav_type_;
  string _sensors_;
  string _turbo_remote_constraints_;

  // | ------------------ Data storage, inputs ------------------ |

  vector<TopicInfo>                                   generic_topic_vec_;
  vector<string>                                      generic_topic_input_vec_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriber_vec_;
  vector<string_info>                                 string_info_vec_;

  vector<node_info> node_info_vec_;

  vector<string> tf_static_list_compare_;
  vector<string> tf_static_list_add_;

  // | ---------------------- Flight timer ---------------------- |

  unsigned long     secs_flown_ = 0;
  rclcpp::Time      last_flight_time_;
  const std::string _time_filename_ = "/tmp/mrs_status_flight_time.txt";
  const std::string _wh_filename_   = "/tmp/mrs_status_wh_drained.txt";

  // | ------------------- Battery measurement ------------------ |

  double            wh_drained_ = 0.0;
  rclcpp::Time      last_got_batt_;
  std::atomic<bool> got_bat_ = false;

  // | -------------------- Switches, states -------------------- |

  bool is_flying_ = false;

  int               sec1_counter_ = 0;
  int               sec5_counter_ = 0;
  std::atomic<bool> initialized_  = false;
};

//}

/* Acquisition() //{ */

Acquisition::Acquisition(rclcpp::NodeOptions options) : mrs_lib::Node("mrs_status_acquisition", options) {

  this->initialize();
}

//}

/* initialize() //{ */

void Acquisition::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  uav_state_ts_          = TopicInfo(node_, 10, BUFFER_LEN_SECS, uav_state_expected_rate_);
  control_manager_ts_    = TopicInfo(node_, 1, BUFFER_LEN_SECS, control_manager_expected_rate_);
  hw_api_odometry_ts_    = TopicInfo(node_, 1, BUFFER_LEN_SECS, hw_api_odometry_expected_rate_);
  hw_api_gnss_ts_        = TopicInfo(node_, 1, BUFFER_LEN_SECS, hw_api_gnss_expected_rate_);
  hw_api_gnss_status_ts_ = TopicInfo(node_, 1, BUFFER_LEN_SECS, hw_api_gnss_status_expected_rate_);
  hw_api_state_ts_       = TopicInfo(node_, 1, BUFFER_LEN_SECS, hw_api_state_expected_rate_);
  hw_api_battery_ts_     = TopicInfo(node_, 1, BUFFER_LEN_SECS, hw_api_battery_expected_rate_);
  hw_api_mag_ts_         = TopicInfo(node_, 1, BUFFER_LEN_SECS, hw_api_mag_expected_rate_);

  prefillUavStatus();

  last_flight_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  last_got_batt_    = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  // | ----------------------- load params ---------------------- |

  mrs_lib::ParamLoader param_loader(node_);

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  std::string platform_config_path;

  param_loader.loadParam("platform_config", platform_config_path);

  if (platform_config_path != "") {
    param_loader.addYamlFile(platform_config_path);
  }

  param_loader.addYamlFileFromParam("config_public");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("uav_type", _uav_type_);
  /* param_loader.loadParam("sensors", _sensors_); */
  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", _turbo_remote_constraints_);
  param_loader.loadParam("mrs_uav_status/enable_profiler", _profiler_enabled_);

  std::vector<string> want_hz;
  param_loader.loadParam("mrs_uav_status/want_hz", want_hz);

  std::vector<string> tf_static_list;
  param_loader.loadParam("mrs_uav_status/tf_static_list", tf_static_list);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.uav_name = _uav_name_;
    uav_status_.uav_type = _uav_type_;
  }

  // | ------------------- want hz handling ------------------- |
  //
  for (unsigned long i = 0; i < want_hz.size(); i++) {
    generic_topic_input_vec_.push_back(want_hz[i]);
  }


  // | ------------------- Static tf handling ------------------- |
  // Static tfs are used to add monitored topics to the generic window, if a tf for a sensor is present, its topic is added

  for (unsigned long i = 0; i < tf_static_list.size(); i++) {

    // this splits the loaded tf_static list into two parts, the first part is intended to be compared with the incoming static tfs, the second one
    // is added to the list of topics if the static tf is present

    std::string::size_type pos = tf_static_list[i].find(' ');
    tf_static_list_compare_.push_back(tf_static_list[i].substr(0, pos));
    tf_static_list_add_.push_back(tf_static_list[i].substr(pos + 1));
  }

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&Acquisition::timerStatus, this);

    timer_status_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(10.0, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Acquisition::timerHostInfo, this);

    timer_host_info_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(general_info_window_rate_, clock_), callback_fcn);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  /* sh_estimation_diag_      = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diag_in"); */

  sh_uav_state_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>(shopts, "~/uav_state_in", &Acquisition::callbackUavState, this);
  sh_tracker_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "~/cmd_tracker_in", &Acquisition::callbackTrackerCommand, this);
  sh_estimator_diag_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diag_in", &Acquisition::callbackEstimationDiaag, this);
  sh_mpc_tracker_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::MpcTrackerDiagnostics>(shopts, "~/mpc_diag_in", &Acquisition::callbackMpcTrackerDiag, this);
  sh_hw_api_status_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in", &Acquisition::callbackHwApiStatus, this);
  sh_hw_api_gnss_      = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, "~/gnss_in", &Acquisition::callbackHwApiGNSS, this);
  sh_throttle_         = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/throttle_in", &Acquisition::callbackThrottle, this);
  sh_mass_estimate_    = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_estimate_in", &Acquisition::callbackMassEstimate, this);
  sh_nominal_mass_     = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_set_in", &Acquisition::callbackNominalMass, this);
  sh_hw_api_gnss_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>(shopts, "~/gnss_status_in", &Acquisition::callbackHwApiGNSSStatus, this);
  sh_battery_state_      = mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>(shopts, "~/battery_in", &Acquisition::callbackBatteryState, this);
  sh_control_manager_diag_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_in", &Acquisition::callbackControlManagerDiag, this);
  sh_gain_manager_diag_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>(shopts, "~/gain_manager_in", &Acquisition::callbackGainManagerDiag, this);
  sh_constraint_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics>(
      shopts, "~/constraint_manager_in", &Acquisition::callbackConstraintManagerDiagnostics, this);
  sh_string_          = mrs_lib::SubscriberHandler<std_msgs::msg::String>(shopts, "~/string_in", &Acquisition::callbackString, this);
  sh_tf_static_       = mrs_lib::SubscriberHandler<tf2_msgs::msg::TFMessage>(shopts, "~/tf_static_in", &Acquisition::callbackTfStatic, this);
  sh_hw_api_odom_     = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odometry_in", &Acquisition::callbackHwApiOdom, this);
  sh_autostart_ready_ = mrs_lib::SubscriberHandler<std_msgs::msg::Bool>(shopts, "~/automatic_start_in", &Acquisition::callbackAutostartReady, this);
  sh_magnetometer_    = mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, "~/mag_in", &Acquisition::callbackMagnetometer, this);

  // | ----------------------- Publishers ---------------------- |

  ph_uav_status_       = mrs_lib::PublisherHandler<mrs_msgs::msg::UavStatus>(node_, "~/uav_status_out");
  ph_uav_status_short_ = mrs_lib::PublisherHandler<mrs_msgs::msg::UavStatusShort>(node_, "~/uav_status_short_out");

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(node_, "Acquisition", _profiler_enabled_);

  // | ---------------------- Flight timer ---------------------- |
  //
  if (boost::filesystem::exists(_time_filename_)) {

    // loads time flown from a tmp file, if it exists, if it does not exists, flight time is set to 0

    ifstream file(_time_filename_);
    string   line;
    getline(file, line);

    try {
      secs_flown_ = stoi(line);
    }
    catch (const invalid_argument &e) {
      secs_flown_ = 0;
    }
    file.close();
  }

  if (boost::filesystem::exists(_wh_filename_)) {

    // loads time flown from a tmp file, if it exists, if it does not exists, flight time is set to 0

    ifstream file(_wh_filename_);
    string   line;
    getline(file, line);

    try {
      wh_drained_ = stod(line);
    }
    catch (const invalid_argument &e) {
      wh_drained_ = 0.0;
    }
    file.close();
  }

  /* Generic topic definitions //{ */

  // TODO this seems like a bad way to do this
  /* vector<string> results; */
  /* split(results, _sensors_, boost::is_any_of(", "), boost::token_compress_on); */

  /* for (unsigned long i = 0; i < results.size(); i++) { */
  /*   if (results[i] == "garmin_down") { */
  /*     generic_topic_input_vec_.push_back("hw_api/distance_sensor Rangefinder 80+"); */

  /*   } else if (results[i] == "garmin_up") { */
  /*     generic_topic_input_vec_.push_back("garmin/range_up Garmin_Up 80+"); */
  /*   } */
  /* } */

  // TODO disabled becase broken in ROS1
  // it crashes cause the tf-based generic subscribers don't know what topic to subscribe
  // setupGenericCallbacks();

  //}

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "[AcquisitionAcquisition]: Node initialized!");

  updateNodeList();
}

//}

/* timerStatus //{ */

void Acquisition::timerStatus() {

  if (!initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerStatus(): spinning");

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("uavStateHandler");
    uavStateHandler();
  }

  sec1_counter_++;
  sec5_counter_++;

  if (sec5_counter_ == 50) {
    sec5_counter_ = 0;
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("updateNodeList");
      updateNodeList();
    }
  }

  if (sec1_counter_ == 10) {

    /* ROS_INFO_ONCE("[%s]: Running data acquisition, publishing uav status.", ros::this_node::getName().c_str()); */

    sec1_counter_ = 0;

    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("hwApDiagHandler");
      hwApiDiagHandler();
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("controlManagerHandler");
      controlManagerHandler();
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("genericTopicHandler");
      genericTopicHandler();
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("stringHandler");
      stringHandler();
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("flightTimeHandler");
      flightTimeHandler();
    }

    {
      std::scoped_lock lock(mutex_status_msg_);

      uav_status_.header.stamp = clock_->now();
      ph_uav_status_.publish(uav_status_);
    }

  } else {

    {
      std::scoped_lock lock(mutex_status_msg_);
      ph_uav_status_short_.publish(uav_status_short_);
    }
  }
}


//}

/* HANDLERS //{ */

/* stringHandler() //{ */

void Acquisition::stringHandler() {

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.custom_string_outputs.clear();
  }

  for (size_t i = 0; i < string_info_vec_.size(); i++) {

    if ((clock_->now() - string_info_vec_[i].last_time).seconds() > 10.0 && !string_info_vec_[i].persistent) {

      string_info_vec_.erase(string_info_vec_.begin() + i);
      i--;

    } else {

      std::scoped_lock lock(mutex_status_msg_);
      uav_status_.custom_string_outputs.push_back(string_info_vec_[i].display_string);
    }
  }
}

//}

//}

/* genericTopicHandler() //{ */

void Acquisition::genericTopicHandler() {

  if (!generic_topic_vec_.empty()) {

    std::vector<mrs_msgs::msg::CustomTopic> custom_topic_vec_out;

    for (size_t i = 0; i < generic_topic_vec_.size(); i++) {
      std::tuple<double, int16_t> rate_color = generic_topic_vec_[i].GetHz();
      mrs_msgs::msg::CustomTopic  custom_topic;
      custom_topic.topic_name  = generic_topic_vec_[i].GetTopicDisplayName();
      custom_topic.topic_hz    = std::get<0>(rate_color);
      custom_topic.topic_color = std::get<1>(rate_color);
      custom_topic_vec_out.push_back(custom_topic);
    }

    {
      std::scoped_lock lock(mutex_status_msg_);
      uav_status_.custom_topics = custom_topic_vec_out;
    }
  }
}

//}

/* uavStateHandler() //{ */

void Acquisition::uavStateHandler() {

  std::tuple<double, int16_t> rate_color = uav_state_ts_.GetHz();
  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.odom_hz          = std::get<0>(rate_color);
    uav_status_.odom_color       = std::get<1>(rate_color);
    uav_status_short_.odom_hz    = uav_status_.odom_hz;
    uav_status_short_.odom_color = uav_status_.odom_color;
  }
}

//}

/* getPort //{ */

int Acquisition::getPort(std::string uri) {

  int  port         = 0;
  bool pre_colon    = false;
  bool start_number = false;

  for (size_t i = 0; i < uri.size(); i++) {

    if (uri[i] == ':') {
      pre_colon = true;
    } else {

      if (pre_colon || start_number) {
        if (uri[i] >= '0' && uri[i] <= '9') {
          start_number = true;
          port         = port * 10 + (uri[i] - '0');
        } else if (start_number) {
          break;
        }
      }
      pre_colon = false;
    }
  }

  return port;
}

//}

/* updateNodeList() //{ */

void Acquisition::updateNodeList() {

  /* vector<node_info> new_node_info_vec_; */

  /* std::vector<std::string> node_names; */
  /* ros::master::getNodes(node_names); */

  /* for (size_t i = 0; i < node_names.size(); i++) { */

  /*   // Get URI of the node */
  /*   XmlRpc::XmlRpcValue args, result, payload; */
  /*   args.setSize(2); */
  /*   args[0] = ""; */
  /*   args[1] = node_names[i]; */
  /*   ros::master::execute("lookupNode", args, result, payload, true); */

  /*   // Make new client of node */
  /*   std::string uri = result[2]; */

  /*   // TODO: make the getPort routine nicer */
  /*   XmlRpc::XmlRpcClient* client = ros::XMLRPCManager::instance()->getXMLRPCClient(ros::master::getHost(), getPort(uri), uri.c_str()); */

  /*   // Get PID of the node */
  /*   XmlRpc::XmlRpcValue request, response; */
  /*   request.setSize(1); */
  /*   /1* request[0] = single_node.node_name; *1/ */
  /*   client->execute("getPid", request, response); */
  /*   int pid = response[2]; */

  /*   ros::XMLRPCManager::instance()->releaseXMLRPCClient(client); */

  /*   if (pid != 0) { */
  /*     node_info tmp_info(node_names[i]); */
  /*     tmp_info.node_pid = pid; */
  /*     new_node_info_vec_.push_back(tmp_info); */
  /*   } */
  /* } */

  /* for (size_t i = 0; i < new_node_info_vec_.size(); i++) { */
  /*   for (size_t j = 0; j < node_info_vec_.size(); j++) { */
  /*     if (new_node_info_vec_[i].node_pid == node_info_vec_[j].node_pid) { */
  /*       new_node_info_vec_[i].last_stime = node_info_vec_[j].last_stime; */
  /*       new_node_info_vec_[i].last_utime = node_info_vec_[j].last_utime; */
  /*     } */
  /*   } */
  /* } */

  /* node_info_vec_ = new_node_info_vec_; */
}

//}

/* nodeCpuLoadHandler() //{ */

void Acquisition::nodeCpuLoadHandler() {

  for (size_t i = 0; i < node_info_vec_.size(); i++) {

    ifstream file("/proc/" + to_string(node_info_vec_[i].node_pid) + "/stat");

    if (file.fail()) {
      // file not found - means process does not exist anymore
      node_info_vec_.erase(node_info_vec_.begin() + i);
      continue;
    }

    string line;
    getline(file, line);
    file.close();

    vector<string> results;
    boost::split(results, line, [](char c) { return c == ' '; });

    long stime;
    long utime;

    try {
      utime = stol(results[13]);
      stime = stol(results[14]);
    }
    catch (const invalid_argument &e) {
      stime = 0;
      utime = 0;
    }

    double user_util = 100.0 * float(utime - node_info_vec_[i].last_utime) / float(total_diff_);
    double sys_util  = 100.0 * float(stime - node_info_vec_[i].last_stime) / float(total_diff_);

    node_info_vec_[i].last_stime = stime;
    node_info_vec_[i].last_utime = utime;

    node_info_vec_[i].node_cpu_usage = cpu_cores_ * (user_util + sys_util);
  }

  sort(node_info_vec_.begin(), node_info_vec_.end(), [](const node_info &a, const node_info &b) -> bool { return a.node_cpu_usage > b.node_cpu_usage; });

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.node_cpu_loads.node_names.clear();
    uav_status_.node_cpu_loads.cpu_loads.clear();

    double cpuload = 0.0;

    mrs_msgs::msg::NodeCpuLoad tmp_node_cpu_load;
    for (size_t i = 0; i < node_info_vec_.size(); i++) {
      tmp_node_cpu_load.cpu_loads.push_back(node_info_vec_[i].node_cpu_usage);
      cpuload += node_info_vec_[i].node_cpu_usage;
      tmp_node_cpu_load.node_names.push_back(node_info_vec_[i].node_name);
    }
    uav_status_.node_cpu_loads = tmp_node_cpu_load;
    uav_status_.cpu_load_total = cpuload;
  }
}

//}

/* controlManagerHandler() //{ */

void Acquisition::controlManagerHandler() {

  std::tuple<double, int16_t> rate_color = control_manager_ts_.GetHz();
  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.control_manager_diag_hz    = std::get<0>(rate_color);
    uav_status_.control_manager_diag_color = std::get<1>(rate_color);
  }
}

//}

/* hwApiDiagHandler() //{ */

void Acquisition::hwApiDiagHandler() {

  std::tuple<double, int16_t> odometry_rate_color    = hw_api_odometry_ts_.GetHz();
  std::tuple<double, int16_t> gnss_rate_color        = hw_api_gnss_ts_.GetHz();
  std::tuple<double, int16_t> gnss_status_rate_color = hw_api_gnss_status_ts_.GetHz();
  std::tuple<double, int16_t> state_rate_color       = hw_api_state_ts_.GetHz();
  std::tuple<double, int16_t> battery_rate_color     = hw_api_battery_ts_.GetHz();


  bool gnss = false;
  if (std::get<0>(gnss_rate_color) > 0.0) {
    gnss = true;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.hw_api_hz             = std::get<0>(odometry_rate_color);
    uav_status_.hw_api_color          = std::get<1>(odometry_rate_color);
    uav_status_.hw_api_gnss_ok        = gnss;
    uav_status_.hw_api_gnss_status_hz = std::get<0>(gnss_status_rate_color);
    uav_status_.hw_api_battery_hz     = std::get<0>(battery_rate_color);
    uav_status_.hw_api_state_hz       = std::get<0>(state_rate_color);
    uav_status_.hw_api_cmd_hz         = std::get<0>(state_rate_color);
  }
}
//}

/* flightTimeHandler() //{ */

void Acquisition::flightTimeHandler() {

  bool null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    null_tracker = uav_status_.null_tracker;
  }

  if (null_tracker) {

    is_flying_ = false;

  } else {

    if (!is_flying_) {

      is_flying_        = true;
      last_flight_time_ = clock_->now();

    } else {

      int secs_passed = int((clock_->now() - last_flight_time_).seconds());

      if (secs_passed > 0) {

        secs_flown_ += secs_passed;
        last_flight_time_ = last_flight_time_ + rclcpp::Duration(secs_passed, clock_->get_clock_type());

        std::ofstream outputFile(_time_filename_);
        outputFile << secs_flown_;
        outputFile.close();
      }
    }
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.secs_flown = secs_flown_;
  }
}

//}

/* timerHostInfo() //{ */

void Acquisition::timerHostInfo() {

  if (!initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerHostInfo(): spinning");

  getCpuLoad();
  getCpuTemperature();
  nodeCpuLoadHandler();
  getMemLoad();
  getCpuFreq();
  getDiskSpace();
}

//}

/* setupGenericCallbacks() //{ */

void Acquisition::setupGenericCallbacks() {

  generic_topic_vec_.clear();
  generic_subscriber_vec_.clear();

  for (size_t i = 0; i < generic_topic_input_vec_.size(); i++) {

    vector<string> results;
    boost::split(results, generic_topic_input_vec_[i], [](char c) { return c == ' '; }); // split the input string into words and put them in results vector
    if (results[2].back() == '+') {
      // TODO handle the + sign
      results[2].pop_back();
    }

    string tmp_string = results[1];
    for (unsigned long j = 2; j < results.size() - 1; j++) {
      tmp_string = tmp_string + " " + results[j];
    }

    try {
      TopicInfo tmp_topic(node_, generic_topic_window_rate_, BUFFER_LEN_SECS, stoi(results[results.size() - 1]), results[0], tmp_string);
      generic_topic_vec_.push_back(tmp_topic);
    }
    catch (const invalid_argument &e) {
    }

    int    id = i; // id to identify which topic called the generic callback
    string topic_name;

    if (generic_topic_vec_[i].GetTopicName().at(0) == '/') {

      topic_name = generic_topic_vec_[i].GetTopicName();

    } else {

      topic_name = "/" + _uav_name_ + "/" + generic_topic_vec_[i].GetTopicName();
    }

    std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback_fcn =
        std::bind(&Acquisition::callbackGeneric, this, std::placeholders::_1, topic_name, id);

    auto tmp_subscriber = node_->create_generic_subscription(topic_name, "std_msgs::msg::Empty", rclcpp::SystemDefaultsQoS(), callback_fcn);

    generic_subscriber_vec_.push_back(tmp_subscriber);
  }
}

//}

/* prefillUavStatus() //{ */

void Acquisition::prefillUavStatus() {

  std::scoped_lock lock(mutex_status_msg_);
  uav_status_.uav_name                = "N/A";
  uav_status_.uav_type                = "N/A";
  uav_status_.uav_mass                = 0.0;
  uav_status_.mass_set                = 0.0;
  uav_status_.control_manager_diag_hz = 0.0;
  uav_status_.controllers.clear();
  uav_status_.gains.clear();
  uav_status_.trackers.clear();
  uav_status_.constraints.clear();
  uav_status_.secs_flown = 0;
  uav_status_.odom_hz    = 0.0;
  uav_status_.odom_x     = 0.0;
  uav_status_.odom_y     = 0.0;
  uav_status_.odom_z     = 0.0;
  uav_status_.odom_hdg   = 0.0;
  uav_status_.odom_frame = "N/A";
  uav_status_.odom_estimators.clear();
  uav_status_.horizontal_estimator = "N/A";
  uav_status_.vertical_estimator   = "N/A";
  uav_status_.heading_estimator    = "N/A";
  uav_status_.agl_estimator        = "N/A";
  uav_status_.max_flight_z         = 0.0;
  uav_status_.cpu_load             = 0.0;
  uav_status_.cpu_ghz              = 0.0;
  uav_status_.cpu_temperature      = 0.0;
  uav_status_.free_ram             = 0.0;
  uav_status_.free_hdd             = 0.0;
  uav_status_.hw_api_hz            = 0.0;
  uav_status_.hw_api_armed         = false;
  uav_status_.hw_api_mode          = "N/A";
  uav_status_.hw_api_gnss_ok       = false;
  uav_status_.hw_api_gnss_qual     = 0.0;
  uav_status_.battery_volt         = 0.0;
  uav_status_.battery_curr         = 0.0;
  uav_status_.thrust               = 0.0;
  uav_status_.mass_estimate        = 0.0;
  uav_status_.mass_set             = 0.0;
  uav_status_.custom_topics.clear();
  uav_status_.custom_string_outputs.clear();
  uav_status_.flying_normally     = false;
  uav_status_.null_tracker        = true;
  uav_status_.have_goal           = false;
  uav_status_.rc_mode             = false;
  uav_status_.tracking_trajectory = false;
  uav_status_.callbacks_enabled   = false;
}

//}

/* CPU/MEM/HDD GET FUNCTIONS //{ */

/* getMemLoad() //{ */

void Acquisition::getMemLoad() {

  ifstream file("/proc/meminfo");
  string   line1, line2, line3, line4;
  getline(file, line1);
  getline(file, line2);
  getline(file, line3);
  getline(file, line4);
  file.close();

  vector<string> results;
  boost::split(results, line1, [](char c) { return c == ' '; });

  double total_ram = 0;
  double free_ram  = 0;
  double buffers   = 0;

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        total_ram = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument &e) {
        total_ram = 0.0;
      }
      break;
    }
  }

  boost::split(results, line3, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        free_ram = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument &e) {
        free_ram = 0.0;
      }
      break;
    }
  }

  boost::split(results, line4, [](char c) { return c == ' '; });

  for (size_t i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {

      try {
        buffers = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument &e) {
        buffers = 0.0;
      }

      break;
    }
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.free_ram  = free_ram + buffers;
    uav_status_.total_ram = total_ram;
  }
}

//}

/* getCpuTemperature() //{ */

void Acquisition::getCpuTemperature() {

  bool breaker       = false;
  long max_temp      = 0;
  int  file_iterator = 0;

  while (!breaker) {
    std::string temp_filename = "/sys/class/thermal/thermal_zone" + std::to_string(file_iterator) + "/temp";
    file_iterator++;

    if (boost::filesystem::exists(temp_filename)) {

      ifstream    file(temp_filename);
      std::string line;
      getline(file, line);
      file.close();
      long temp = 0;

      try {
        temp = stol(line);
      }
      catch (const invalid_argument &e) {
        temp = 0;
      }
      if (temp > max_temp) {
        max_temp = temp;
      }
    } else {
      breaker = true;
    }
    if (file_iterator > 10) {
      breaker = true;
    }
  }


  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.cpu_temperature = float(max_temp) / 1000;
  }
}

//}

/* getCpuLoad() //{ */

void Acquisition::getCpuLoad() {

  ifstream file("/proc/stat");
  string   line;
  getline(file, line);
  file.close();

  vector<string> results;
  boost::split(results, line, [](char c) { return c == ' '; });

  long idle;
  long non_idle;
  long total;

  try {
    idle     = stol(results[5]) + stol(results[6]);
    non_idle = stol(results[2]) + stol(results[3]) + stol(results[4]) + stol(results[7]) + stol(results[8]) + stol(results[9]);
    total    = idle + non_idle;
  }
  catch (const invalid_argument &e) {
    idle     = 0;
    non_idle = 0;
    total    = 0;
  }

  last_total_diff_ = total_diff_;
  total_diff_      = total - last_total_;
  long idle_diff   = idle - last_idle_;

  double cpu_load = 100 * (double(total_diff_ - idle_diff) / double(total_diff_));

  last_total_ = total;
  last_idle_  = idle;

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.cpu_load = cpu_load;
  }
}

//}

/* getCpuFreq() //{ */

void Acquisition::getCpuFreq() {

  ifstream file("/sys/devices/system/cpu/online");
  string   line;
  getline(file, line);
  file.close();

  vector<string> results;
  boost::split(results, line, [](char c) { return c == '-'; });


  try {
    cpu_cores_ = stoi(results[1]) + 1;
  }
  catch (const invalid_argument &e) {
    cpu_cores_ = 1;
  }

  long cpu_freq = 0;

  for (int i = 0; i < cpu_cores_; i++) {
    string   filename = "/sys/devices/system/cpu/cpu" + to_string(i) + "/cpufreq/scaling_cur_freq";
    ifstream file(filename.c_str());
    getline(file, line);
    file.close();
    try {
      cpu_freq += stol(line);
    }
    catch (const invalid_argument &e) {
      cpu_freq = 0;
    }
  }

  double avg_cpu_ghz = (double(cpu_freq) / cpu_cores_) / 1048576;

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.cpu_ghz = avg_cpu_ghz;
  }
}

//}

/* getDiskSpace() //{ */

void Acquisition::getDiskSpace() {

  boost::filesystem::space_info si = boost::filesystem::space(".");

  int gigas = round(si.available / 104857600);

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.free_hdd = gigas;
  }
}

//}

//}

/* CALLBACKS //{ */

/* callbackUavState() //{ */

void Acquisition::callbackUavState(const mrs_msgs::msg::UavState::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  uav_state_ts_.Count();

  double heading;

  try {
    heading = mrs_lib::AttitudeConverter(msg->pose.orientation).getHeading();
  }
  catch (...) {
    heading = 0;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.odom_x     = msg->pose.position.x;
    uav_status_.odom_y     = msg->pose.position.y;
    uav_status_.odom_z     = msg->pose.position.z;
    uav_status_.odom_hdg   = heading;
    uav_status_.odom_frame = msg->header.frame_id;

    uav_status_short_.odom_x   = msg->pose.position.x;
    uav_status_short_.odom_y   = msg->pose.position.y;
    uav_status_short_.odom_z   = msg->pose.position.z;
    uav_status_short_.odom_hdg = heading;
  }
}

//}

/* callbackTrackerCommand() //{ */

void Acquisition::callbackTrackerCommand(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.cmd_x   = msg->position.x;
    uav_status_.cmd_y   = msg->position.y;
    uav_status_.cmd_z   = msg->position.z;
    uav_status_.cmd_hdg = msg->heading;

    uav_status_short_.cmd_x   = msg->position.x;
    uav_status_short_.cmd_y   = msg->position.y;
    uav_status_short_.cmd_z   = msg->position.z;
    uav_status_short_.cmd_hdg = msg->heading;
  }
}

//}

/* callbackEstimationDiaag() //{ */

void Acquisition::callbackEstimationDiaag(const mrs_msgs::msg::EstimationDiagnostics::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.max_flight_z    = msg->max_flight_z;
    uav_status_.odom_estimators = msg->switchable_state_estimators;

    for (size_t i = 0; i < uav_status_.odom_estimators.size(); i++) {
      if ((uav_status_.odom_estimators[i] == msg->current_state_estimator) && i != 0) {

        // put the active estimator as first in the vector
        std::swap(uav_status_.odom_estimators[0], uav_status_.odom_estimators[i]);
      }
    }

    uav_status_.horizontal_estimator = msg->estimator_horizontal;
    uav_status_.vertical_estimator   = msg->estimator_vertical;
    uav_status_.heading_estimator    = msg->estimator_heading;
    uav_status_.agl_estimator        = msg->estimator_agl_height;
  }
}

//}

/* callbackMpcTrackerDiag() //{ */

void Acquisition::callbackMpcTrackerDiag(const mrs_msgs::msg::MpcTrackerDiagnostics::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.avoiding_collision          = msg->avoiding_collision;
    uav_status_.collision_avoidance_enabled = msg->collision_avoidance_active;
    uav_status_.num_other_uavs              = uint16_t(msg->avoidance_active_uavs.size());
  }
}

//}

/* callbackHwApiStatus() //{ */

void Acquisition::callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  hw_api_state_ts_.Count();

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.hw_api_armed = msg->armed;
    uav_status_.hw_api_mode  = msg->mode;
  }
}

//}

/* callbackBatteryState() //{ */

void Acquisition::callbackBatteryState(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  hw_api_battery_ts_.Count();

  if (!got_bat_) {
    // first time we got the battery message, set the last_bat time to now.
    got_bat_       = true;
    last_got_batt_ = clock_->now();
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.battery_volt = msg->voltage;
    uav_status_.battery_curr = msg->current;

    double bat_dt  = (clock_->now() - last_got_batt_).seconds();
    last_got_batt_ = clock_->now();

    wh_drained_ += uav_status_.battery_volt * uav_status_.battery_curr * (bat_dt / 3600);
    uav_status_.battery_wh_drained = wh_drained_;

    ofstream outputFile(_wh_filename_);
    outputFile << wh_drained_;
    outputFile.close();
  }
}

//}

/* callbackThrottle() //{ */

void Acquisition::callbackThrottle(const std_msgs::msg::Float64::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.thrust = msg->data;
  }
}

//}

/* callbackMassEstimate() //{ */

void Acquisition::callbackMassEstimate(const std_msgs::msg::Float64::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.mass_estimate = msg->data;
  }
}

//}

/* callbackNominalMass() //{ */

void Acquisition::callbackNominalMass(const std_msgs::msg::Float64::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.mass_set = msg->data;
  }
}

//}

/* callbackHwApiGNSS() //{ */

void Acquisition::callbackHwApiGNSS(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  hw_api_gnss_ts_.Count();

  double gnss_qual = (msg->position_covariance[0] + msg->position_covariance[4] + msg->position_covariance[8]) / 3;

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.hw_api_gnss_qual = gnss_qual;
  }
}

//}

/* callbackHwApiGNSSStatus() //{ */

void Acquisition::callbackHwApiGNSSStatus(const mrs_msgs::msg::GpsInfo::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  hw_api_gnss_status_ts_.Count();

  double gnss_acc = (msg->h_acc + msg->v_acc) / 2;

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.hw_api_gnss_fix_type = msg->fix_type;
    uav_status_.hw_api_gnss_num_sats = msg->satellites_visible;
    uav_status_.hw_api_gnss_pos_acc  = gnss_acc;
  }
}

//}

/* callbackHwApiOdom() //{ */

void Acquisition::callbackHwApiOdom([[maybe_unused]] const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  hw_api_odometry_ts_.Count();
}

//}

/* callbackControlManagerDiag() //{ */

void Acquisition::callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "callbackControlManagerDiag(): getting data");

  control_manager_ts_.Count();

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.trackers.clear();
    uav_status_.controllers.clear();

    for (size_t i = 0; i < msg->available_trackers.size(); i++) {
      if (msg->human_switchable_trackers[i]) {
        uav_status_.trackers.push_back(msg->available_trackers[i]);
      }
    }

    for (size_t i = 0; i < msg->available_controllers.size(); i++) {
      if (msg->human_switchable_controllers[i]) {
        uav_status_.controllers.push_back(msg->available_controllers[i]);
      }
    }

    if (std::find(uav_status_.trackers.begin(), uav_status_.trackers.end(), msg->active_tracker) != uav_status_.trackers.end()) {

      // active tracker is in the trackers vector, swap it to the first position
      for (size_t i = 0; i < uav_status_.trackers.size(); i++) {
        if ((uav_status_.trackers[i] == msg->active_tracker) && i != 0) {
          // put the active estimator as first in the vector
          std::swap(uav_status_.trackers[0], uav_status_.trackers[i]);
        }
      }

    } else {
      // active tracker is not in the trackers vector, put it there
      uav_status_.trackers.insert(uav_status_.trackers.begin(), msg->active_tracker);
    }


    if (std::find(uav_status_.controllers.begin(), uav_status_.controllers.end(), msg->active_controller) != uav_status_.controllers.end()) {

      // active controller is in the controllers vecotor, swap it to the first position
      for (size_t i = 0; i < uav_status_.controllers.size(); i++) {
        if ((uav_status_.controllers[i] == msg->active_controller) && i != 0) {
          // put the active estimator as first in the vector
          std::swap(uav_status_.controllers[0], uav_status_.controllers[i]);
        }
      }

    } else {
      // active tracker is not in the controllers vecotor, put it there
      uav_status_.controllers.insert(uav_status_.controllers.begin(), msg->active_controller);
    }

    uav_status_.flying_normally     = msg->flying_normally;
    uav_status_.have_goal           = msg->tracker_status.have_goal;
    uav_status_.rc_mode             = msg->joystick_active;
    uav_status_.tracking_trajectory = msg->tracker_status.tracking_trajectory;
    uav_status_.callbacks_enabled   = msg->tracker_status.callbacks_enabled;

    if (msg->active_tracker == "NullTracker") {
      uav_status_.null_tracker = true;
    } else {
      uav_status_.null_tracker = false;
    }
  }
}

//}

/* callbackGainManagerDiag() //{ */

void Acquisition::callbackGainManagerDiag(const mrs_msgs::msg::GainManagerDiagnostics::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.gains = msg->available;

    for (size_t i = 0; i < uav_status_.gains.size(); i++) {
      if ((uav_status_.gains[i] == msg->current_name) && i != 0) {
        // put the active estimator as first in the vector
        std::swap(uav_status_.gains[0], uav_status_.gains[i]);
      }
    }
  }
}

//}

/* callbackAutostartReady() //{ */

void Acquisition::callbackAutostartReady(const std_msgs::msg::Bool::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.automatic_start_can_takeoff = msg->data;
  }
}

//}

/* callbackMagnetometer() //{ */

void Acquisition::callbackMagnetometer(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  hw_api_mag_ts_.Count();

  {
    std::scoped_lock lock(mutex_status_msg_);

    std::tuple<double, int16_t> rate_color = hw_api_mag_ts_.GetHz();
    uav_status_.mag_norm                   = 10000 * (sqrt(pow(msg->magnetic_field.x, 2) + pow(msg->magnetic_field.y, 2) + pow(msg->magnetic_field.z, 2)));
    uav_status_.mag_norm_hz                = std::get<0>(rate_color);
  }
}

//}

/* callbackConstraintManagerDiagnostics() //{ */

void Acquisition::callbackConstraintManagerDiagnostics(const mrs_msgs::msg::ConstraintManagerDiagnostics::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.constraints = msg->available;

    for (size_t i = 0; i < uav_status_.constraints.size(); i++) {
      if ((uav_status_.constraints[i] == msg->current_name) && i != 0) {

        // put the active estimator as first in the vector
        std::swap(uav_status_.constraints[0], uav_status_.constraints[i]);
      }
    }
  }
}

//}

/* callbackTfStatic() //{ */

void Acquisition::callbackTfStatic(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  bool got_new_tf_static = false;

  for (size_t i = 0; i < msg->transforms.size(); i++) {

    std::string tmp        = msg->transforms[i].child_frame_id;
    std::size_t pos        = tmp.find("/");       // find the / in uav1/something
    std::string uav_name   = tmp.substr(0, pos);  // cut out the uav name, so we can discard tfs from other drones (mostly for simulation)
    std::string frame_name = tmp.substr(pos + 1); // cut off the uav1/ from the tf_static name
                                                  //
    // TODO fix this mess
    for (size_t j = 0; j < tf_static_list_compare_.size(); j++) {

      if (tf_static_list_compare_[j] == frame_name && uav_name == _uav_name_) {

        std::scoped_lock lock(mutex_status_msg_);

        if (std::find(generic_topic_input_vec_.begin(), generic_topic_input_vec_.end(), tf_static_list_add_[j]) == generic_topic_input_vec_.end()) {
          generic_topic_input_vec_.push_back(tf_static_list_add_[j]);
        }

        got_new_tf_static = true;
      }
    }
  }

  // if (got_new_tf_static) {
  //   setupGenericCallbacks();
  // }
}
//}

/* callbackString() //{ */

void Acquisition::callbackString(const std_msgs::msg::String::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  std::string pub_name = ""; // TODO this information got lost from ROS1
  std::string msg_str  = msg->data;

  std::stringstream                  ss(msg_str);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string>           msg_vector(begin, end);

  bool   params_read = false;
  size_t iterator    = 0;

  std::string id         = "";
  bool        persistent = false;

  while (!params_read) {

    if (msg_vector[iterator] == "-id") {

      if (iterator + 1 < msg_vector.size()) {
        id = msg_vector[iterator + 1];
        msg_vector.erase(msg_vector.begin() + iterator);
        msg_vector.erase(msg_vector.begin() + iterator);
        continue;
      }

    } else if (msg_vector[iterator] == "-p") {
      msg_vector.erase(msg_vector.begin() + iterator);
      persistent = true;
      continue;

    } else if (msg_vector[iterator].at(0) != '-') {
      params_read = true;
      continue;
    }
    iterator++;
    if (iterator >= msg_vector.size()) {
      params_read = true;
    }
  }

  msg_str       = "";
  bool first_it = true;

  for (size_t i = 0; i < msg_vector.size(); i++) {

    if (first_it) {
      first_it = false;
    } else {
      msg_str += " ";
    }

    msg_str += msg_vector[i];
  }

  bool contains = false;

  /* uav_status_.custom_string_outputs */
  for (unsigned long i = 0; i < string_info_vec_.size(); i++) {
    if (string_info_vec_[i].publisher_name == pub_name && string_info_vec_[i].id == id) {
      contains                           = true;
      string_info_vec_[i].display_string = msg_str;
      string_info_vec_[i].persistent     = persistent;
      string_info_vec_[i].last_time      = clock_->now();
      break;
    }
  }

  if (!contains) {
    string_info tmp(clock_->now(), pub_name, msg_str, id, persistent);
    string_info_vec_.push_back(tmp);
  }
}

//}

/* callbackGeneric() //{ */

void Acquisition::callbackGeneric([[maybe_unused]] std::shared_ptr<rclcpp::SerializedMessage> msg, [[maybe_unused]] const std::string topic, const int id) {

  if (!initialized_) {
    return;
  }

  generic_topic_vec_[id].Count();
}

//}

//}

} // namespace mrs_uav_status

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_status::Acquisition)

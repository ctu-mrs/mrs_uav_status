/* INCLUDES //{ */

/* #include <status_window.h> */
#include <mrs_msgs/NodeCpuLoad.h>
#include <commons.h>
#include <mrs_msgs/CustomTopic.h>
#include <mrs_lib/profiler.h>

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <boost/filesystem.hpp>

#include <ros/master.h>
#include <ros/xmlrpc_manager.h>
#include <XmlRpcClient.h>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/UavStatusShort.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/ControllerDiagnostics.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/CustomTopic.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <mrs_lib/mutex.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>

#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/GpsInfo.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

#include <tf2_msgs/TFMessage.h>

#include <cmath>

using namespace std;
using topic_tools::ShapeShifter;

//}

/* class Acquisition //{ */

class Acquisition {

public:
  Acquisition();


private:
  ros::NodeHandle nh_;

  std::mutex mutex_general_info_thread_;
  std::mutex mutex_status_msg_;

  mrs_msgs::UavStatus      uav_status_;
  mrs_msgs::UavStatusShort uav_status_short_;

  // | ------------------------- Timers ------------------------- |

  ros::Timer status_timer_;

  void statusTimer(const ros::TimerEvent& event);

  // | ------------------------ Utility ----------------------- |

  int getPort(std::string uri);

  // | ------------------------ Callbacks ----------------------- |

  void uavStateCallback(const mrs_msgs::UavStateConstPtr& msg);
  void trackerCommandCallback(const mrs_msgs::TrackerCommandConstPtr& msg);
  void estimationDiagCallback(const mrs_msgs::EstimationDiagnosticsConstPtr& msg);
  void mpcDiagCallback(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg);
  void hwApiStatusCallback(const mrs_msgs::HwApiStatusConstPtr& msg);
  void batteryCallback(const sensor_msgs::BatteryStateConstPtr& msg);
  void throttleCallback(const std_msgs::Float64ConstPtr& msg);
  void callbackMassEstimate(const std_msgs::Float64ConstPtr& msg);
  void callbackMassSet(const std_msgs::Float64ConstPtr& msg);
  void controlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  void gainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg);
  void constraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg);
  void stringCallback(const ros::MessageEvent<std_msgs::String const>& event);
  void tfStaticCallback(const tf2_msgs::TFMessage& msg);
  void hwApiGnssCallback(const sensor_msgs::NavSatFixConstPtr& msg);
  void hwApiGnssStatusCallback(const mrs_msgs::GpsInfoConstPtr& msg);
  void hwApiOdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void automaticStartCallback_(const std_msgs::BoolConstPtr& msg);
  void magCallback_(const sensor_msgs::MagneticFieldConstPtr& msg);

  // generic callback, for any topic, to monitor its rate
  void genericCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name, const int id);

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
  double hw_api_cmd_expected_rate_         = 100.0;
  double hw_api_battery_expected_rate_     = 0.5;
  double hw_api_mag_expected_rate_         = 10;

  TopicInfo uav_state_ts_{10, BUFFER_LEN_SECS, uav_state_expected_rate_};
  TopicInfo control_manager_ts_{1, BUFFER_LEN_SECS, control_manager_expected_rate_};
  TopicInfo hw_api_odometry_ts_{1, BUFFER_LEN_SECS, hw_api_odometry_expected_rate_};
  TopicInfo hw_api_gnss_ts_{1, BUFFER_LEN_SECS, hw_api_gnss_expected_rate_};
  TopicInfo hw_api_gnss_status_ts_{1, BUFFER_LEN_SECS, hw_api_gnss_status_expected_rate_};
  TopicInfo hw_api_state_ts_{1, BUFFER_LEN_SECS, hw_api_state_expected_rate_};
  TopicInfo hw_api_cmd_ts_{1, BUFFER_LEN_SECS, hw_api_cmd_expected_rate_};
  TopicInfo hw_api_battery_ts_{1, BUFFER_LEN_SECS, hw_api_battery_expected_rate_};
  TopicInfo hw_api_mag_ts_{1, BUFFER_LEN_SECS, hw_api_mag_expected_rate_};

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

  void generalInfoThread();

  void flightTimeHandler();

  void setupGenericCallbacks();

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  std::thread general_info_thread_;
  bool        run_thread_ = true;


  // | ----------------------- Subscribers ---------------------- |

  ros::Subscriber uav_state_subscriber_;
  ros::Subscriber cmd_position_subscriber_;
  ros::Subscriber estimation_diag_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;
  ros::Subscriber hw_api_status_subscriber_;
  ros::Subscriber hw_api_gnss_subscriber_;
  ros::Subscriber hw_api_gnss_status_subscriber_;
  ros::Subscriber hw_api_odometry_subscriber_;
  ros::Subscriber mass_estimate_subscriber_;
  ros::Subscriber mass_set_subscriber_;
  ros::Subscriber throttle_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Subscriber control_manager_subscriber_;
  ros::Subscriber gain_manager_subscriber_;
  ros::Subscriber constraint_manager_subscriber_;
  ros::Subscriber string_subscriber_;
  ros::Subscriber set_service_subscriber_;
  ros::Subscriber tf_static_subscriber_;
  ros::Subscriber automatic_start_subscriber_;
  ros::Subscriber mag_subscriber_;

  // | ----------------------- Publishers ---------------------- |

  ros::Publisher uav_status_publisher_;
  ros::Publisher uav_status_short_publisher_;

  // | -------------------- UAV configuration ------------------- |

  string _uav_name_;
  string _uav_type_;
  string _run_type_;
  string _sensors_;
  string _turbo_remote_constraints_;

  // | ------------------ Data storage, inputs ------------------ |

  vector<TopicInfo>       generic_topic_vec_;
  vector<string>          generic_topic_input_vec_;
  vector<ros::Subscriber> generic_subscriber_vec_;
  vector<string_info>     string_info_vec_;

  vector<node_info> node_info_vec_;

  vector<string> tf_static_list_compare_;
  vector<string> tf_static_list_add_;

  // | ---------------------- Flight timer ---------------------- |

  unsigned long     secs_flown_ = 0;
  ros::Time         last_flight_time_;
  const std::string _time_filename_ = "/tmp/mrs_status_flight_time.txt";
  const std::string _wh_filename_   = "/tmp/mrs_status_wh_drained.txt";

  // | ------------------- Battery measurement ------------------ |

  double    wh_drained_ = 0.0;
  ros::Time last_got_batt_;
  bool      got_bat_ = false;

  // | -------------------- Switches, states -------------------- |

  bool is_flying_ = false;

  int  sec1_counter_ = 0;
  int  sec5_counter_ = 0;
  bool initialized_  = false;
};

//}

/* Acquisition() //{ */

Acquisition::Acquisition() {

  // initialize node and create no handle
  nh_ = ros::NodeHandle("~");

  // | ---------------------- Param loader ---------------------- |

  prefillUavStatus();

  mrs_lib::ParamLoader param_loader(nh_, "Acquisition");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("uav_type", _uav_type_);
  param_loader.loadParam("run_type", _run_type_);
  param_loader.loadParam("sensors", _sensors_);
  param_loader.loadParam("turbo_remote_constraints", _turbo_remote_constraints_);
  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  std::vector<string> want_hz;
  param_loader.loadParam("want_hz", want_hz);

  std::vector<string> tf_static_list;
  param_loader.loadParam("tf_static_list", tf_static_list);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Acquisition]: Could not load all parameters!");
    ros::shutdown();
  } else {
    ROS_INFO("[Acquisition]: All params loaded!");
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

  status_timer_ = nh_.createTimer(ros::Rate(10), &Acquisition::statusTimer, this);

  // | ----------------------- Subscribers ---------------------- |

  uav_state_subscriber_          = nh_.subscribe("uav_state_in", 10, &Acquisition::uavStateCallback, this, ros::TransportHints().tcpNoDelay());
  cmd_position_subscriber_       = nh_.subscribe("cmd_tracker_in", 10, &Acquisition::trackerCommandCallback, this, ros::TransportHints().tcpNoDelay());
  estimation_diag_subscriber_    = nh_.subscribe("estimation_diag_in", 10, &Acquisition::estimationDiagCallback, this, ros::TransportHints().tcpNoDelay());
  mpc_diag_subscriber_           = nh_.subscribe("mpc_diag_in", 10, &Acquisition::mpcDiagCallback, this, ros::TransportHints().tcpNoDelay());
  hw_api_status_subscriber_      = nh_.subscribe("hw_api_status_in", 10, &Acquisition::hwApiStatusCallback, this, ros::TransportHints().tcpNoDelay());
  throttle_subscriber_           = nh_.subscribe("throttle_in", 10, &Acquisition::throttleCallback, this, ros::TransportHints().tcpNoDelay());
  mass_estimate_subscriber_      = nh_.subscribe("mass_estimate_in", 10, &Acquisition::callbackMassEstimate, this, ros::TransportHints().tcpNoDelay());
  mass_set_subscriber_           = nh_.subscribe("mass_set_in", 10, &Acquisition::callbackMassSet, this, ros::TransportHints().tcpNoDelay());
  hw_api_gnss_subscriber_        = nh_.subscribe("gnss_in", 10, &Acquisition::hwApiGnssCallback, this, ros::TransportHints().tcpNoDelay());
  hw_api_gnss_status_subscriber_ = nh_.subscribe("gnss_status_in", 10, &Acquisition::hwApiGnssStatusCallback, this, ros::TransportHints().tcpNoDelay());
  battery_subscriber_            = nh_.subscribe("battery_in", 10, &Acquisition::batteryCallback, this, ros::TransportHints().tcpNoDelay());
  control_manager_subscriber_    = nh_.subscribe("control_manager_in", 10, &Acquisition::controlManagerCallback, this, ros::TransportHints().tcpNoDelay());
  gain_manager_subscriber_       = nh_.subscribe("gain_manager_in", 10, &Acquisition::gainManagerCallback, this, ros::TransportHints().tcpNoDelay());
  constraint_manager_subscriber_ =
      nh_.subscribe("constraint_manager_in", 10, &Acquisition::constraintManagerCallback, this, ros::TransportHints().tcpNoDelay());
  string_subscriber_          = nh_.subscribe("string_in", 10, &Acquisition::stringCallback, this, ros::TransportHints().tcpNoDelay());
  tf_static_subscriber_       = nh_.subscribe("tf_static_in", 100, &Acquisition::tfStaticCallback, this, ros::TransportHints().tcpNoDelay());
  hw_api_odometry_subscriber_ = nh_.subscribe("odometry_in", 10, &Acquisition::hwApiOdometryCallback, this, ros::TransportHints().tcpNoDelay());
  automatic_start_subscriber_ = nh_.subscribe("automatic_start_in", 10, &Acquisition::automaticStartCallback_, this, ros::TransportHints().tcpNoDelay());
  mag_subscriber_             = nh_.subscribe("mag_in", 10, &Acquisition::magCallback_, this, ros::TransportHints().tcpNoDelay());


  // | ----------------------- Publishers ---------------------- |

  uav_status_publisher_       = nh_.advertise<mrs_msgs::UavStatus>("uav_status_out", 1);
  uav_status_short_publisher_ = nh_.advertise<mrs_msgs::UavStatusShort>("uav_status_short_out", 1);

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(nh_, "Acquisition", _profiler_enabled_);

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
    catch (const invalid_argument& e) {
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
    catch (const invalid_argument& e) {
      wh_drained_ = 0.0;
    }
    file.close();
  }

  /* Generic topic definitions //{ */

  vector<string> results;
  split(results, _sensors_, boost::is_any_of(", "), boost::token_compress_on);


  for (unsigned long i = 0; i < results.size(); i++) {
    if (results[i] == "garmin_down") {
      generic_topic_input_vec_.push_back("hw_api/distance_sensor Rangefinder 80+");

    } else if (results[i] == "garmin_up") {
      generic_topic_input_vec_.push_back("garmin/range_up Garmin_Up 80+");
    }
  }

  setupGenericCallbacks();

  //}

  // This thread is for the general info window, it fetches CPU frequency, RAM info, Disk space info and CPU load asynchronously
  general_info_thread_ = std::thread{&Acquisition::generalInfoThread, this};

  initialized_ = true;
  ROS_INFO("[AcquisitionAcquisition]: Node initialized!");

  updateNodeList();
}

//}

/* statusTimer //{ */

void Acquisition::statusTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!initialized_) {
    return;
  }

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
      uav_status_.header.stamp = ros::Time::now();
      uav_status_publisher_.publish(uav_status_);
    }

  } else {

    {
      std::scoped_lock lock(mutex_status_msg_);
      uav_status_short_publisher_.publish(uav_status_short_);
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

    if ((ros::Time::now() - string_info_vec_[i].last_time).toSec() > 10.0 && !string_info_vec_[i].persistent) {

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

    std::vector<mrs_msgs::CustomTopic> custom_topic_vec_out;

    for (size_t i = 0; i < generic_topic_vec_.size(); i++) {
      std::tuple<double, int16_t> rate_color = generic_topic_vec_[i].GetHz();
      mrs_msgs::CustomTopic       custom_topic;
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
  for (int i = 0; i < uri.size(); i++) {
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
  vector<node_info> new_node_info_vec_;

  std::vector<std::string> node_names;
  ros::master::getNodes(node_names);

  for (size_t i = 0; i < node_names.size(); i++) {

    // Get URI of the node
    XmlRpc::XmlRpcValue args, result, payload;
    args.setSize(2);
    args[0] = "";
    args[1] = node_names[i];
    ros::master::execute("lookupNode", args, result, payload, true);

    // Make new client of node
    std::string uri = result[2];

    // TODO: make the getPort routine nicer
    XmlRpc::XmlRpcClient* client = ros::XMLRPCManager::instance()->getXMLRPCClient(ros::master::getHost(), getPort(uri), uri.c_str());

    // Get PID of the node
    XmlRpc::XmlRpcValue request, response;
    request.setSize(1);
    /* request[0] = single_node.node_name; */
    client->execute("getPid", request, response);
    int pid = response[2];

    ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);

    if (pid != 0) {
      node_info tmp_info(node_names[i]);
      tmp_info.node_pid = pid;
      new_node_info_vec_.push_back(tmp_info);
    }
  }

  for (size_t i = 0; i < new_node_info_vec_.size(); i++) {
    for (size_t j = 0; j < node_info_vec_.size(); j++) {
      if (new_node_info_vec_[i].node_pid == node_info_vec_[j].node_pid) {
        new_node_info_vec_[i].last_stime = node_info_vec_[j].last_stime;
        new_node_info_vec_[i].last_utime = node_info_vec_[j].last_utime;
      }
    }
  }

  node_info_vec_ = new_node_info_vec_;
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
    catch (const invalid_argument& e) {
      stime = 0;
      utime = 0;
    }

    double user_util = 100.0 * float(utime - node_info_vec_[i].last_utime) / float(total_diff_);
    double sys_util  = 100.0 * float(stime - node_info_vec_[i].last_stime) / float(total_diff_);

    node_info_vec_[i].last_stime = stime;
    node_info_vec_[i].last_utime = utime;

    node_info_vec_[i].node_cpu_usage = cpu_cores_ * (user_util + sys_util);
  }

  sort(node_info_vec_.begin(), node_info_vec_.end(), [](const node_info& a, const node_info& b) -> bool { return a.node_cpu_usage > b.node_cpu_usage; });

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.node_cpu_loads.node_names.clear();
    uav_status_.node_cpu_loads.cpu_loads.clear();

    double cpuload = 0.0;

    mrs_msgs::NodeCpuLoad tmp_node_cpu_load;
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
  std::tuple<double, int16_t> cmd_rate_color         = hw_api_cmd_ts_.GetHz();
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
      last_flight_time_ = ros::Time::now();

    } else {

      int secs_passed = int((ros::Time::now() - last_flight_time_).toSec());

      if (secs_passed > 0) {

        secs_flown_ += secs_passed;
        last_flight_time_ = last_flight_time_ + ros::Duration(secs_passed);

        ofstream outputFile(_time_filename_);
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

/* generalInfoThread() //{ */

void Acquisition::generalInfoThread() {

  ros::Time last_time;

  while (true) {

    double interval = (ros::Time::now() - last_time).toSec();

    if (interval >= 1.0 / double(general_info_window_rate_)) {

      std::scoped_lock lock(mutex_general_info_thread_);

      last_time = ros::Time::now();

      {
        mrs_lib::Routine profiler_routine = profiler_.createRoutine("generalInfoThread");
        getCpuLoad();
        getCpuTemperature();
        nodeCpuLoadHandler();
        getMemLoad();
        getCpuFreq();
        getDiskSpace();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(int(general_info_window_rate_ * 10)));
  }
}

//}

/* setupGenericCallbacks() //{ */

void Acquisition::setupGenericCallbacks() {

  generic_topic_vec_.clear();
  generic_subscriber_vec_.clear();

  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;  // generic callback

  for (size_t i = 0; i < generic_topic_input_vec_.size(); i++) {

    vector<string> results;
    boost::split(results, generic_topic_input_vec_[i], [](char c) { return c == ' '; });  // split the input string into words and put them in results vector
    if (results[2].back() == '+') {
      // TODO handle the + sign
      results[2].pop_back();
    }

    string tmp_string = results[1];
    for (unsigned long j = 2; j < results.size() - 1; j++) {
      tmp_string = tmp_string + " " + results[j];
    }

    try {
      TopicInfo tmp_topic(generic_topic_window_rate_, BUFFER_LEN_SECS, stoi(results[results.size() - 1]), results[0], tmp_string);
      generic_topic_vec_.push_back(tmp_topic);
    }
    catch (const invalid_argument& e) {
    }


    int    id = i;  // id to identify which topic called the generic callback
    string topic_name;

    if (generic_topic_vec_[i].GetTopicName().at(0) == '/') {

      topic_name = generic_topic_vec_[i].GetTopicName();

    } else {

      topic_name = "/" + _uav_name_ + "/" + generic_topic_vec_[i].GetTopicName();
    }

    callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { genericCallback(msg, topic_name, id); };
    ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 10, callback);

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

  double total_ram;
  double free_ram;
  double used_ram;
  double buffers;

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        total_ram = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument& e) {
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
      catch (const invalid_argument& e) {
        free_ram = 0.0;
      }
      break;
    }
  }

  boost::split(results, line4, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        buffers = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument& e) {
        buffers = 0.0;
      }
      break;
    }
  }

  used_ram = total_ram - (free_ram + buffers);

  int    tmp_color = GREEN;
  double ram_ratio = used_ram / total_ram;
  if (ram_ratio > 0.8) {
    tmp_color = RED;
  } else if (ram_ratio > 0.6) {
    tmp_color = YELLOW;
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
      catch (const invalid_argument& e) {
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
  catch (const invalid_argument& e) {
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
  catch (const invalid_argument& e) {
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
    catch (const invalid_argument& e) {
      cpu_freq = 0;
    }
  }

  double avg_cpu_ghz = double(cpu_freq / cpu_cores_) / 1048576;

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

/* uavStateCallback() //{ */

void Acquisition::uavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {

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

/* trackerCommandCallback() //{ */

void Acquisition::trackerCommandCallback(const mrs_msgs::TrackerCommandConstPtr& msg) {

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

/* estimationDiagCallback() //{ */

void Acquisition::estimationDiagCallback(const mrs_msgs::EstimationDiagnosticsConstPtr& msg) {

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

/* mpcDiagCallback() //{ */

void Acquisition::mpcDiagCallback(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg) {

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

/* hwApiStatusCallback() //{ */

void Acquisition::hwApiStatusCallback(const mrs_msgs::HwApiStatusConstPtr& msg) {

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

/* batteryCallback() //{ */

void Acquisition::batteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {

  if (!initialized_) {
    return;
  }

  hw_api_battery_ts_.Count();

  if (!got_bat_) {
    // first time we got the battery message, set the last_bat time to now.
    got_bat_       = true;
    last_got_batt_ = ros::Time::now();
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.battery_volt = msg->voltage;
    uav_status_.battery_curr = msg->current;
    double bat_dt            = (ros::Time::now() - last_got_batt_).toSec();
    last_got_batt_           = ros::Time::now();
    wh_drained_ += uav_status_.battery_volt * uav_status_.battery_curr * (bat_dt / 3600);
    uav_status_.battery_wh_drained = wh_drained_;

    ofstream outputFile(_wh_filename_);
    outputFile << wh_drained_;
    outputFile.close();
  }
}

//}

/* throttleCallback() //{ */

void Acquisition::throttleCallback(const std_msgs::Float64ConstPtr& msg) {

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

void Acquisition::callbackMassEstimate(const std_msgs::Float64ConstPtr& msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.mass_estimate = msg->data;
  }
}

//}

/* callbackMassSet() //{ */

void Acquisition::callbackMassSet(const std_msgs::Float64ConstPtr& msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.mass_set = msg->data;
  }
}

//}

/* hwApiGnssCallback() //{ */

void Acquisition::hwApiGnssCallback(const sensor_msgs::NavSatFixConstPtr& msg) {

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

/* hwApiGnssStatusCallback() //{ */

void Acquisition::hwApiGnssStatusCallback(const mrs_msgs::GpsInfoConstPtr& msg) {

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

/* hwApiOdometryCallback() //{ */

void Acquisition::hwApiOdometryCallback(const nav_msgs::OdometryConstPtr& msg) {

  if (!initialized_) {
    return;
  }
  hw_api_odometry_ts_.Count();
}

//}

/* controlManagerCallback() //{ */

void Acquisition::controlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!initialized_) {
    return;
  }

  control_manager_ts_.Count();

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.trackers.clear();
    uav_status_.controllers.clear();

    for (int i = 0; i < msg->available_trackers.size(); i++) {
      if (msg->human_switchable_trackers[i]) {
        uav_status_.trackers.push_back(msg->available_trackers[i]);
      }
    }

    for (int i = 0; i < msg->available_controllers.size(); i++) {
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

/* gainManagerCallback() //{ */

void Acquisition::gainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg) {

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

/* automaticStartCallback_() //{ */

void Acquisition::automaticStartCallback_(const std_msgs::BoolConstPtr& msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_.automatic_start_can_takeoff = msg->data;
  }
}

//}

/* magCallback_() //{ */

void Acquisition::magCallback_(const sensor_msgs::MagneticFieldConstPtr& msg) {

  if (!initialized_) {
    return;
  }
  hw_api_mag_ts_.Count();
  {
    std::scoped_lock            lock(mutex_status_msg_);
    std::tuple<double, int16_t> rate_color = hw_api_mag_ts_.GetHz();
    uav_status_.mag_norm                   = 10000 * (sqrt(pow(msg->magnetic_field.x, 2) + pow(msg->magnetic_field.y, 2) + pow(msg->magnetic_field.z, 2)));
    uav_status_.mag_norm_hz                = std::get<0>(rate_color);
    /* ROS_INFO_STREAM("[%s]: Absolute: "  << 10000*(sqrt(pow(msg->magnetic_field.x, 2) + pow(msg->magnetic_field.y, 2) + pow(msg->magnetic_field.z, 2))) << "
     * X: " <<  10000*(msg->magnetic_field.x) << " Y: " <<  10000*(msg->magnetic_field.y) << " Z: " <<  10000*(msg->magnetic_field.z)); */
  }
}

//}

/* constraintManagerCallback() //{ */

void Acquisition::constraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {

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

/* tfStaticCallback() //{ */

void Acquisition::tfStaticCallback(const tf2_msgs::TFMessage& msg) {

  if (!initialized_) {
    return;
  }

  bool got_new_tf_static = false;

  for (unsigned long i = 0; i < msg.transforms.size(); i++) {

    std::string tmp        = msg.transforms[i].child_frame_id;
    std::size_t pos        = tmp.find("/");        // find the / in uav1/something
    std::string uav_name   = tmp.substr(0, pos);   // cut out the uav name, so we can discard tfs from other drones (mostly for simulation)
    std::string frame_name = tmp.substr(pos + 1);  // cut off the uav1/ from the tf_static name
    // TODO fix this mess
    for (unsigned long j = 0; j < tf_static_list_compare_.size(); j++) {
      if (tf_static_list_compare_[j] == frame_name && uav_name == _uav_name_) {
        std::scoped_lock lock(mutex_status_msg_);
        if (std::find(generic_topic_input_vec_.begin(), generic_topic_input_vec_.end(), tf_static_list_add_[j]) == generic_topic_input_vec_.end())
          generic_topic_input_vec_.push_back(tf_static_list_add_[j]);
        got_new_tf_static = true;
      }
    }
  }
  if (got_new_tf_static) {
    setupGenericCallbacks();
  }
}
//}

/* stringCallback() //{ */

void Acquisition::stringCallback(const ros::MessageEvent<std_msgs::String const>& event) {

  if (!initialized_) {
    return;
  }
  std::string pub_name = event.getPublisherName();
  std::string msg_str  = event.getMessage()->data;

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
      string_info_vec_[i].last_time      = ros::Time::now();
      break;
    }
  }

  if (!contains) {
    string_info tmp(pub_name, msg_str, id, persistent);
    string_info_vec_.push_back(tmp);
  }
}
//}

/* genericCallback() //{ */

void Acquisition::genericCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name, const int id) {

  if (!initialized_) {
    return;
  }

  generic_topic_vec_[id].Count();
}

//}

//}

/* main() //{ */

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrs_uav_status");

  Acquisition acquisition;

  while (ros::ok()) {

    ros::spin();
    return 0;
  }
}

//

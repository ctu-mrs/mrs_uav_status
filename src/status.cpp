/* INCLUDES //{ */

#include <status_window.h>
#include <menu.h>
#include <input_box.h>
#include <commons.h>
#include <mrs_lib/profiler.h>

#include <iostream>
#include <fstream>
#include <thread>
#include <boost/filesystem.hpp>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/AttitudeCommand.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>

#include <std_msgs/String.h>

#include <mrs_lib/mutex.h>

#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/BatteryState.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

#include <tf2_msgs/TFMessage.h>

using namespace std;
using topic_tools::ShapeShifter;

//}

typedef enum
{
  STANDARD,
  REMOTE,
  MAIN_MENU,
  GOTO_MENU,
} status_state;

/* class Status //{ */

class Status {

public:
  Status();

  string _colorscheme_;
  bool   _rainbow_;
  bool   _light_ = false;

private:
  ros::NodeHandle nh_;

  std::mutex mutex_general_info_thread_;

  // | ------------------------- Timers ------------------------- |

  ros::Timer status_timer_;
  ros::Timer resize_timer_;

  void statusTimer(const ros::TimerEvent& event);
  void resizeTimer(const ros::TimerEvent& event);

  // | ------------------------ Callbacks ----------------------- |

  void uavStateCallback(const mrs_msgs::UavStateConstPtr& msg);
  void mavrosStateCallback(const mavros_msgs::StateConstPtr& msg);
  void cmdAttitudeCallback(const mrs_msgs::AttitudeCommandConstPtr& msg);
  void batteryCallback(const sensor_msgs::BatteryStateConstPtr& msg);
  void controlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  void gainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg);
  void constraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg);
  void stringCallback(const ros::MessageEvent<std_msgs::String const>& event);
  void setServiceCallback(const std_msgs::String& msg);
  void tfStaticCallback(const tf2_msgs::TFMessage& msg);
  void mavrosGlobalCallback(const sensor_msgs::NavSatFixConstPtr& msg);

  // generic callback, for any topic, to monitor its rate
  void genericCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name, const int id);

  // | --------------------- Print routines --------------------- |

  void printLimitedInt(WINDOW* win, int y, int x, string str_in, int num, int limit);
  void printLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit);
  void printLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit);
  void printServiceResult(bool success, string msg);
  void printDebug(string msg);
  void printHelp();

  void printNoData(WINDOW* win, int y, int x);
  void printNoData(WINDOW* win, int y, int x, string text);


  // | ------------------------- Windows ------------------------ |

  void genericTopicHandler(WINDOW* win, double rate, short color, int topic);
  void uavStateHandler(WINDOW* win, double rate, short color, int topic);
  void mavrosStateHandler(WINDOW* win, double rate, short color, int topic);
  void controlManagerHandler(WINDOW* win, double rate, short color, int topic);
  void stringHandler(WINDOW* win, double rate, short color, int topic);

  double uav_state_window_rate_       = 10;
  double control_manager_window_rate_ = 1;
  double mavros_state_window_rate_    = 1;
  double general_info_window_rate_    = 1;
  double generic_topic_window_rate_   = 1;

  // General info window
  void printCpuLoad(WINDOW* win);
  void printCpuFreq(WINDOW* win);
  void printMemLoad(WINDOW* win);
  void printDiskSpace(WINDOW* win);

  long last_idle_  = 0;
  long last_total_ = 0;
  long last_gigas_ = 0;

  // Custom windows
  StatusWindow* uav_state_window_;
  vector<topic> uav_state_topic_;

  StatusWindow* mavros_state_window_;
  vector<topic> mavros_state_topic_;

  StatusWindow* control_manager_window_;
  vector<topic> control_manager_topic_;

  StatusWindow*       string_window_;
  vector<string_info> string_info_vec_;
  vector<topic>       string_topic_;

  // Vanilla windows
  WINDOW* top_bar_window_;
  WINDOW* bottom_window_;
  WINDOW* general_info_window_;
  WINDOW* debug_window_;

  ros::Time bottom_window_clear_time_ = ros::Time::now();

  bool help_active_ = false;

  StatusWindow* generic_topic_window_;

  // | ---------------------- Misc routines --------------------- |

  void generalInfoThread();

  void flightTimeHandler(WINDOW* win);

  void remoteHandler(int key, WINDOW* win);

  void setupGenericCallbacks();

  std::string callTerminal(const char* cmd);

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  std::thread general_info_thread_;
  bool        run_thread_ = true;


  // | ---------------------- Menu routines --------------------- |

  void setupMainMenu();
  bool mainMenuHandler(int key_in);

  void setupGotoMenu();
  bool gotoMenuHandler(int key_in);

  // | ----------------------- Subscribers ---------------------- |

  ros::Subscriber uav_state_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber mavros_global_subscriber_;
  ros::Subscriber attitude_cmd_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Subscriber control_manager_subscriber_;
  ros::Subscriber gain_manager_subscriber_;
  ros::Subscriber constraint_manager_subscriber_;
  ros::Subscriber string_subscriber_;
  ros::Subscriber set_service_subscriber_;
  ros::Subscriber tf_static_subscriber_;

  // | --------------------- Service Clients -------------------- |

  ros::ServiceClient service_goto_reference_;
  ros::ServiceClient service_goto_fcu_;
  ros::ServiceClient service_set_constraints_;
  ros::ServiceClient service_set_gains_;
  ros::ServiceClient service_set_controller_;
  ros::ServiceClient service_hover_;

  // | -------------------- UAV configuration ------------------- |

  string _uav_name_;
  string _nato_name_;
  string _uav_type_;
  double _uav_mass_;
  string _sensors_;
  bool   _pixgarm_;

  // | ------------------ Data storage, inputs ------------------ |
  mavros_msgs::State        mavros_state_;
  mrs_msgs::AttitudeCommand cmd_attitude_;
  sensor_msgs::NavSatFix    mavros_global_;
  sensor_msgs::BatteryState battery_;

  mrs_msgs::UavState                     uav_state_;
  mrs_msgs::ControlManagerDiagnostics    control_manager_;
  mrs_msgs::GainManagerDiagnostics       gain_manager_;
  mrs_msgs::ConstraintManagerDiagnostics constraint_manager_;

  vector<topic>           generic_topic_vec_;
  vector<string>          generic_topic_input_vec_;
  vector<ros::Subscriber> generic_subscriber_vec_;

  vector<Menu> menu_vec_;
  vector<Menu> submenu_vec_;

  vector<service> service_vec_;
  vector<string>  service_input_vec_;
  vector<string>  main_menu_text_;
  vector<string>  constraints_text_;
  vector<string>  gains_text_;
  vector<string>  controllers_text_;

  vector<double>   goto_double_vec_;
  vector<string>   goto_menu_text_;
  vector<InputBox> goto_menu_inputs_;

  vector<string> tf_static_list_compare_;
  vector<string> tf_static_list_add_;

  string old_constraints;

  // | ---------------------- Flight timer ---------------------- |

  unsigned long     secs_flown = 0;
  ros::Time         last_flight_time_;
  const std::string _time_filename_ = "/tmp/mrs_status_flight_time.txt";

  // | -------------------- Switches, states -------------------- |

  bool remote_hover_ = false;
  bool turbo_remote_ = false;


  bool is_flying_   = false;
  bool NullTracker_ = true;

  status_state state = STANDARD;
  int          cols_, lines_;

  int  hz_counter_  = 0;
  bool initialized_ = false;
};

//}

/* Status() //{ */

Status::Status() {

  // initialize node and create no handle
  nh_ = ros::NodeHandle("~");

  // | ---------------------- Param loader ---------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "Status");

  string tmp_uav_mass;

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("nato_name", _nato_name_);
  param_loader.loadParam("uav_type", _uav_type_);
  param_loader.loadParam("uav_mass", _uav_mass_);
  param_loader.loadParam("sensors", _sensors_);
  param_loader.loadParam("pixgarm", _pixgarm_);

  param_loader.loadParam("colorscheme", _colorscheme_);
  param_loader.loadParam("rainbow", _rainbow_);

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  std::vector<string> want_hz;
  param_loader.loadParam("want_hz", want_hz);

  std::vector<string> tf_static_list;
  param_loader.loadParam("tf_static_list", tf_static_list);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Status]: Could not load all parameters!");
    ros::shutdown();
  } else {
    ROS_INFO("[Status]: All params loaded!");
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

  status_timer_ = nh_.createTimer(ros::Rate(10), &Status::statusTimer, this);
  resize_timer_ = nh_.createTimer(ros::Rate(1), &Status::resizeTimer, this);

  // | ----------------------- Subscribers ---------------------- |

  uav_state_subscriber_          = nh_.subscribe("uav_state_in", 1, &Status::uavStateCallback, this, ros::TransportHints().tcpNoDelay());
  mavros_state_subscriber_       = nh_.subscribe("mavros_state_in", 1, &Status::mavrosStateCallback, this, ros::TransportHints().tcpNoDelay());
  attitude_cmd_subscriber_       = nh_.subscribe("cmd_attitude_in", 1, &Status::cmdAttitudeCallback, this, ros::TransportHints().tcpNoDelay());
  mavros_global_subscriber_      = nh_.subscribe("mavros_global_in", 1, &Status::mavrosGlobalCallback, this, ros::TransportHints().tcpNoDelay());
  battery_subscriber_            = nh_.subscribe("battery_in", 1, &Status::batteryCallback, this, ros::TransportHints().tcpNoDelay());
  control_manager_subscriber_    = nh_.subscribe("control_manager_in", 1, &Status::controlManagerCallback, this, ros::TransportHints().tcpNoDelay());
  gain_manager_subscriber_       = nh_.subscribe("gain_manager_in", 1, &Status::gainManagerCallback, this, ros::TransportHints().tcpNoDelay());
  constraint_manager_subscriber_ = nh_.subscribe("constraint_manager_in", 1, &Status::constraintManagerCallback, this, ros::TransportHints().tcpNoDelay());
  string_subscriber_             = nh_.subscribe("string_in", 1, &Status::stringCallback, this, ros::TransportHints().tcpNoDelay());
  set_service_subscriber_        = nh_.subscribe("set_service_in", 1, &Status::setServiceCallback, this, ros::TransportHints().tcpNoDelay());
  tf_static_subscriber_          = nh_.subscribe("tf_static_in", 100, &Status::tfStaticCallback, this, ros::TransportHints().tcpNoDelay());

  // | ------------------------ Services ------------------------ |
  //
  service_goto_reference_  = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("reference_out");
  service_goto_fcu_        = nh_.serviceClient<mrs_msgs::Vec4>("goto_fcu_out");
  service_set_constraints_ = nh_.serviceClient<mrs_msgs::String>("set_constraints_out");
  service_set_gains_       = nh_.serviceClient<mrs_msgs::String>("set_gains_out");
  service_set_controller_  = nh_.serviceClient<mrs_msgs::String>("set_controller_out");
  service_hover_           = nh_.serviceClient<std_srvs::Trigger>("hover_out");

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(nh_, "Status", _profiler_enabled_);

  // Loads the default GoTo value
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(2.0);
  goto_double_vec_.push_back(1.57);

  // Loads the default menu options
  service_input_vec_.push_back("uav_manager/land Land");
  service_input_vec_.push_back("uav_manager/land_home Land Home");
  service_input_vec_.push_back("uav_manager/takeoff Takeoff");

  // | ---------------------- Flight timer ---------------------- |
  //
  if (boost::filesystem::exists(_time_filename_)) {

    // loads time flown from a tmp file, if it exists, if it does not exists, flight time is set to 0

    ifstream file(_time_filename_);
    string   line;
    getline(file, line);

    try {
      secs_flown = stoi(line);
    }
    catch (const invalid_argument& e) {
      secs_flown = 0;
    }
    file.close();
  }

  /* Generic topic definitions //{ */

  vector<string> results;
  split(results, _sensors_, boost::is_any_of(", "), boost::token_compress_on);

  if (_pixgarm_) {
    generic_topic_input_vec_.push_back("mavros/distance_sensor/garmin Garmin_pix 80+");
  }

  for (unsigned long i = 0; i < results.size(); i++) {
    if (results[i] == "garmin_down" && _pixgarm_ == false) {
      generic_topic_input_vec_.push_back("garmin/range Garmin_Down 80+");

    } else if (results[i] == "garmin_up") {
      generic_topic_input_vec_.push_back("garmin/range_up Garmin_Up 80+");

      /* } else if (results[i] == "realsense_brick") { */
      /*   generic_topic_input_vec_.push_back("rs_d435/depth/camera_info Realsense_Brick 25+"); */

      /* } else if (results[i] == "realsense_front") { */
      /*   generic_topic_input_vec_.push_back("rs_d435/depth/camera_info Realsense_Front 25+"); */

      /* } else if (results[i] == "bluefox_brick") { */
      /*   generic_topic_input_vec_.push_back("bluefox_brick/camera_info Bluefox_Brick 25+"); */

      /* } else if (results[i] == "bluefox_optflow") { */
      /*   generic_topic_input_vec_.push_back("bluefox_optflow/camera_info Bluefox_Optflow 60+"); */
      /*   generic_topic_input_vec_.push_back("optic_flow/velocity Optic_flow 60+"); */

      /* } else if (results[i] == "trinocular_thermal") { */
      /*   generic_topic_input_vec_.push_back("thermal/top/rgb_image Thermal_Top 15+"); */
      /*   generic_topic_input_vec_.push_back("thermal/middle/rgb_image Thermal_Middle 15+"); */
      /*   generic_topic_input_vec_.push_back("thermal/bottom/rgb_image Thermal_Bottom 15+"); */

      /* } else if (results[i] == "rplidar") { */
      /*   generic_topic_input_vec_.push_back("rplidar/scan Rplidar 10+"); */
    }
  }

  setupGenericCallbacks();

  //}

  // --------------------------------------------------------------
  // |            Window creation and topic association           |
  // --------------------------------------------------------------

  uav_state_topic_.push_back(topic{100.0, uav_state_window_rate_});

  uav_state_window_ = new StatusWindow(6, 26, 5, 1, uav_state_topic_, uav_state_window_rate_);

  control_manager_topic_.push_back(topic{10.0, control_manager_window_rate_});
  control_manager_topic_.push_back(topic{1.0, control_manager_window_rate_});
  control_manager_topic_.push_back(topic{1.0, control_manager_window_rate_});

  control_manager_window_ = new StatusWindow(4, 26, 1, 1, control_manager_topic_, control_manager_window_rate_);

  mavros_state_topic_.push_back(topic{100.0, mavros_state_window_rate_});
  mavros_state_topic_.push_back(topic{1.0, mavros_state_window_rate_});
  mavros_state_topic_.push_back(topic{100.0, mavros_state_window_rate_});
  mavros_state_topic_.push_back(topic{100.0, mavros_state_window_rate_});

  mavros_state_window_ = new StatusWindow(6, 25, 5, 27, mavros_state_topic_, mavros_state_window_rate_);

  generic_topic_window_ = new StatusWindow(10, 25, 1, 52, generic_topic_vec_, generic_topic_window_rate_);

  string_window_ = new StatusWindow(10, 32, 1, 77, string_topic_, generic_topic_window_rate_);

  general_info_window_ = newwin(4, 25, 1, 27);

  top_bar_window_ = newwin(1, 120, 0, 1);
  bottom_window_  = newwin(1, 120, 11, 1);

  debug_window_ = newwin(20, 120, 12, 1);

  initialized_ = true;
  ROS_INFO("[Status]: Node initialized!");

  // This thread is for the general info window, it fetches CPU frequency, RAM info, Disk space info and CPU load asynchronously
  general_info_thread_ = std::thread{&Status::generalInfoThread, this};
}

//}

/* resizeTimer //{ */

void Status::resizeTimer([[maybe_unused]] const ros::TimerEvent& event) {

  // TODO resizing causes screen flicker, should be called just before general refresh.
  // TODO display breaks when screen is resized to very small number of cols

  char                     command[50] = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string              response    = callTerminal(command);
  std::vector<std::string> results;
  boost::split(results, response, [](char c) { return c == 'x'; });

  int cols, lines;

  try {
    cols  = stoi(results[0]);
    lines = stoi(results[1]);
  }

  catch (const invalid_argument& e) {
    cols  = 0;
    lines = 0;
  }
  if (cols_ != cols || lines_ != lines) {
    lines_ = lines;
    cols_  = cols;

    if (cols > 30) {
      resize_term(lines_, cols_);
    }
  }
}


//}

/* statusTimer //{ */

void Status::statusTimer([[maybe_unused]] const ros::TimerEvent& event) {

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("uavStateHandler");
    uav_state_window_->Redraw(&Status::uavStateHandler, _light_, this);
  }

  hz_counter_++;

  if (hz_counter_ == 10) {

    hz_counter_ = 0;

    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("mavrosStateHandler");
      mavros_state_window_->Redraw(&Status::mavrosStateHandler, _light_, this);
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("controlManagerHandler");
      control_manager_window_->Redraw(&Status::controlManagerHandler, _light_, this);
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("genericTopicHandler");
      generic_topic_window_->Redraw(&Status::genericTopicHandler, _light_, this);
    }
    {
      mrs_lib::Routine profiler_routine = profiler_.createRoutine("stringHandler");
      string_window_->Redraw(&Status::stringHandler, _light_, this);
    }
  }

  {
    std::scoped_lock lock(mutex_general_info_thread_);
    wrefresh(general_info_window_);
  }

  wclear(top_bar_window_);

  if ((ros::Time::now() - bottom_window_clear_time_).toSec() > 3.0) {
    wclear(bottom_window_);
  }

  flightTimeHandler(top_bar_window_);

  printHelp();

  int key_in = getch();

  switch (state) {

    case STANDARD:

      switch (key_in) {

        case 'R':
          remote_hover_ = false;
          state         = REMOTE;
          break;

        case 'm':
          setupMainMenu();
          state = MAIN_MENU;
          break;

        case 'g':
          setupGotoMenu();
          state = GOTO_MENU;
          break;

        case 'h':
          help_active_ = !help_active_;
          break;

        default:
          flushinp();
          break;
      }

      break;

    case REMOTE:
      flushinp();
      remoteHandler(key_in, top_bar_window_);
      if (key_in == 'R' || key_in == KEY_ESC) {

        if (turbo_remote_) {

          turbo_remote_ = false;
          mrs_msgs::String string_service;
          string_service.request.value = old_constraints;
          service_set_constraints_.call(string_service);
          printServiceResult(string_service.response.success, string_service.response.message);
        }

        state = STANDARD;
      }
      break;

    case MAIN_MENU:
      flushinp();
      if (mainMenuHandler(key_in)) {
        state = STANDARD;
      }
      break;

    case GOTO_MENU:
      flushinp();
      if (gotoMenuHandler(key_in)) {
        state = STANDARD;
      }
      break;
  }

  wrefresh(top_bar_window_);
  wrefresh(bottom_window_);
  refresh();
}


//}

/* HANDLERS //{ */


/* mainMenuHandler() //{ */

bool Status::mainMenuHandler(int key_in) {

  /* SUBMENU IS OPEN //{ */

  if (!submenu_vec_.empty()) {
    // SUBMENU IS OPEN

    menu_vec_[0].iterate(main_menu_text_, -1, true);
    optional<tuple<int, int>> ret;

    switch (submenu_vec_[0].getId()) {
      case 1:
        // CONSTRAINTS
        ret = submenu_vec_[0].iterate(constraints_text_, key_in, true);

        if (ret.has_value()) {

          int line = get<0>(ret.value());
          int key  = get<1>(ret.value());

          if (line == 666 && key == 666) {

            submenu_vec_.clear();
            return false;

          } else if (key == KEY_ENT) {

            mrs_msgs::String string_service;
            string_service.request.value = constraints_text_[line];
            service_set_constraints_.call(string_service);
            printServiceResult(string_service.response.success, string_service.response.message);

            submenu_vec_.clear();
            return true;
          }
        }
        break;

      case 2:
        // GAINS
        ret = submenu_vec_[0].iterate(gains_text_, key_in, true);

        if (ret.has_value()) {

          int line = get<0>(ret.value());
          int key  = get<1>(ret.value());

          if (line == 666 && key == 666) {

            submenu_vec_.clear();
            return false;

          } else if (key == KEY_ENT) {

            mrs_msgs::String string_service;
            string_service.request.value = gains_text_[line];
            service_set_gains_.call(string_service);
            printServiceResult(string_service.response.success, string_service.response.message);

            submenu_vec_.clear();
            return true;
          }
        }
        break;

      case 3:
        // CONTROLLERS
        ret = submenu_vec_[0].iterate(controllers_text_, key_in, true);

        if (ret.has_value()) {

          int line = get<0>(ret.value());
          int key  = get<1>(ret.value());

          if (line == 666 && key == 666) {

            submenu_vec_.clear();
            return false;

          } else if (key == KEY_ENT) {

            mrs_msgs::String string_service;
            string_service.request.value = controllers_text_[line];
            service_set_controller_.call(string_service);
            printServiceResult(string_service.response.success, string_service.response.message);

            submenu_vec_.clear();
            return true;
          }
        }
        break;
    }

    return false;

    //}

    /* NORMAL CASE //{ */

  } else {
    // NORMAL CASE - NO SUBMENU

    optional<tuple<int, int>> ret = menu_vec_[0].iterate(main_menu_text_, key_in, true);

    if (ret.has_value()) {

      int line = get<0>(ret.value());
      int key  = get<1>(ret.value());

      if (line == 666 && key == 666) {

        menu_vec_.clear();
        return true;

      } else if (key == KEY_ENT) {

        if (line < service_vec_.size()) {

          std_srvs::Trigger trig;
          service_vec_[line].service_client.call(trig);
          printServiceResult(trig.response.success, trig.response.message);

          menu_vec_.clear();
          return true;

        } else if (line == main_menu_text_.size() - 3) {
          // SET CONSTRAINTS

          constraints_text_.clear();

          for (unsigned long i = 0; i < constraint_manager_.available.size(); i++) {
            constraints_text_.push_back(constraint_manager_.available[i]);
          }

          int x;
          int y;
          int rows;
          int cols;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          Menu menu(x, 31 + cols, constraints_text_, 1);
          submenu_vec_.push_back(menu);

        } else if (line == main_menu_text_.size() - 2) {
          // SET GAINS

          gains_text_.clear();

          for (unsigned long i = 0; i < gain_manager_.available.size(); i++) {
            gains_text_.push_back(gain_manager_.available[i]);
          }

          int x;
          int y;
          int rows;
          int cols;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          Menu menu(x, 31 + cols, gains_text_, 2);
          submenu_vec_.push_back(menu);

        } else if (line == main_menu_text_.size() - 1) {
          // SET CONTROLLER

          controllers_text_.clear();
          controllers_text_.push_back("So3Controller");
          controllers_text_.push_back("MpcController");

          int x;
          int y;
          int rows;
          int cols;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          Menu menu(x, 31 + cols, controllers_text_, 3);
          submenu_vec_.push_back(menu);

        } else {
          printServiceResult(false, "undefined");
        }
      }
    }
    return false;
  }

  //}
}

//}

/* gotoMenuHandler() //{ */

bool Status::gotoMenuHandler(int key_in) {

  optional<tuple<int, int>> ret = menu_vec_[0].iterate(goto_menu_text_, key_in, false);

  if (ret.has_value()) {
    int line = get<0>(ret.value());
    int key  = get<1>(ret.value());

    if (line == 666 && key == 666) {

      menu_vec_.clear();
      return true;

    } else if (key == KEY_ENT) {

      goto_double_vec_[0] = goto_menu_inputs_[0].getDouble();
      goto_double_vec_[1] = goto_menu_inputs_[1].getDouble();
      goto_double_vec_[2] = goto_menu_inputs_[2].getDouble();
      goto_double_vec_[3] = goto_menu_inputs_[3].getDouble();

      mrs_msgs::ReferenceStampedSrv reference;

      reference.request.reference.position.x = goto_double_vec_[0];
      reference.request.reference.position.y = goto_double_vec_[1];
      reference.request.reference.position.z = goto_double_vec_[2];
      reference.request.reference.heading    = goto_double_vec_[3];
      reference.request.header.frame_id      = uav_state_.header.frame_id;
      reference.request.header.stamp         = ros::Time::now();

      service_goto_reference_.call(reference);

      printServiceResult(reference.response.success, reference.response.message);

      menu_vec_.clear();

      return true;

    } else if (line < goto_menu_inputs_.size()) {

      goto_menu_inputs_[line].Process(key);
    }
  }


  for (unsigned long i = 0; i < goto_menu_inputs_.size(); i++) {
    if (i == menu_vec_[0].getLine()) {
      goto_menu_inputs_[i].Print(i + 1, true);
    } else {
      goto_menu_inputs_[i].Print(i + 1, false);
    }
  }

  wrefresh(menu_vec_[0].getWin());
}

//}

/* remoteHandler() //{ */

void Status::remoteHandler(int key, WINDOW* win) {

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, 0, 20, "REMOTE MODE IS ACTIVE, YOU HAVE CONTROL");

  if (turbo_remote_) {
    wattron(win, A_BLINK);
    mvwprintw(win, 0, 12, "!TURBO!");
    mvwprintw(win, 0, 60, "!TURBO!");
    wattroff(win, A_BLINK);
  }

  wattroff(win, COLOR_PAIR(RED));

  mrs_msgs::Vec4    goal;
  mrs_msgs::String  string_service;
  std_srvs::Trigger trig;

  goal.request.goal[0] = 0.0;
  goal.request.goal[1] = 0.0;
  goal.request.goal[2] = 0.0;
  goal.request.goal[3] = 0.0;

  switch (key) {

    case 'w':
    case 'k':
    case KEY_UP:
      goal.request.goal[0] = 2.0;

      if (turbo_remote_) {
        goal.request.goal[0] = 5.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 's':
    case 'j':
    case KEY_DOWN:
      goal.request.goal[0] = -2.0;

      if (turbo_remote_) {
        goal.request.goal[0] = -5.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'a':
    case 'h':
    case KEY_LEFT:
      goal.request.goal[1] = 2.0;

      if (turbo_remote_) {
        goal.request.goal[1] = 5.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'd':
    case 'l':
    case KEY_RIGHT:
      goal.request.goal[1] = -2.0;

      if (turbo_remote_) {
        goal.request.goal[1] = -5.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'r':
      goal.request.goal[2] = 1.0;

      if (turbo_remote_) {
        goal.request.goal[2] = 2.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'f':
      goal.request.goal[2] = -1.0;

      if (turbo_remote_) {
        goal.request.goal[2] = -2.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'q':
      goal.request.goal[3] = 0.5;

      if (turbo_remote_) {
        goal.request.goal[3] = 1.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'e':
      goal.request.goal[3] = -0.5;

      if (turbo_remote_) {
        goal.request.goal[3] = -1.0;
      }

      service_goto_fcu_.call(goal);
      remote_hover_ = true;
      break;

    case 'T':

      if (turbo_remote_) {

        turbo_remote_                = false;
        string_service.request.value = old_constraints;
        service_set_constraints_.call(string_service);
        printServiceResult(string_service.response.success, string_service.response.message);

      } else {

        turbo_remote_                = true;
        old_constraints              = constraint_manager_.current_name;
        string_service.request.value = constraint_manager_.available[constraint_manager_.available.size() - 1];
        service_set_constraints_.call(string_service);
        printServiceResult(string_service.response.success, string_service.response.message);
      }

      break;

    default:
      if (remote_hover_) {

        service_hover_.call(trig);
        remote_hover_ = false;
      }
      break;
  }
  wattroff(win, A_BOLD);
}

//}

/* stringHandler() //{ */

void Status::stringHandler(WINDOW* win, double rate, short color, int topic) {

  if (string_info_vec_.empty()) {
    wclear(win);
  }

  for (unsigned long i = 0; i < string_info_vec_.size(); i++) {

    if ((ros::Time::now() - string_info_vec_[i].last_time).toSec() > 10.0) {
      string_info_vec_.erase(string_info_vec_.begin() + i);
    } else {

      printLimitedString(win, 1 + (3 * i), 1, string_info_vec_[i].publisher_name + ": ", 30);

      int    tmp_color          = NORMAL;
      bool   blink              = false;
      string tmp_display_string = string_info_vec_[i].display_string;

      if (tmp_display_string.at(0) == '-') {

        if (tmp_display_string.at(1) == 'r') {
          tmp_color = RED;
        } else if (tmp_display_string.at(1) == 'R') {
          tmp_color = RED;
          blink     = true;
        }

        else if (tmp_display_string.at(1) == 'y') {
          tmp_color = YELLOW;
        } else if (tmp_display_string.at(1) == 'Y') {
          tmp_color = YELLOW;
          blink     = true;
        }

        else if (tmp_display_string.at(1) == 'g') {
          tmp_color = GREEN;
        } else if (tmp_display_string.at(1) == 'G') {
          tmp_color = GREEN;
          blink     = true;
        }

        if (tmp_color != NORMAL) {
          tmp_display_string.erase(0, 3);
        }
      }

      if (blink) {
        wattron(win, A_BLINK);
      }

      wattron(win, COLOR_PAIR(tmp_color));
      printLimitedString(win, 2 + (3 * i), 1, tmp_display_string, 30);
      wattroff(win, COLOR_PAIR(tmp_color));
      wattroff(win, A_BLINK);
    }
  }
}

//}

//}

/* genericTopicHandler() //{ */

void Status::genericTopicHandler(WINDOW* win, double rate, short color, int topic) {

  if (!generic_topic_vec_.empty()) {

    printLimitedString(win, 1 + topic, 1, generic_topic_vec_[topic].topic_display_name, 15);
    printLimitedDouble(win, 1 + topic, 16, "%5.1f Hz", rate, 1000);

  } else {

    wclear(win);
  }
}

//}

/* uavStateHandler() //{ */

void Status::uavStateHandler(WINDOW* win, double rate, [[maybe_unused]] short color, [[maybe_unused]] int topic) {

  printLimitedDouble(win, 0, 12, "Odom %5.1f Hz", rate, 1000);

  if (rate == 0) {

    printNoData(win, 0, 1);

  } else {

    double heading;

    try {
      heading = mrs_lib::AttitudeConverter(uav_state_.pose.orientation).getHeading();
    }
    catch (...) {
      heading = 0;
    }

    printLimitedDouble(win, 1, 1, "X %7.2f", uav_state_.pose.position.x, 1000);
    printLimitedDouble(win, 2, 1, "Y %7.2f", uav_state_.pose.position.y, 1000);
    printLimitedDouble(win, 3, 1, "Z %7.2f", uav_state_.pose.position.z, 1000);
    printLimitedDouble(win, 4, 1, "hdg %5.2f", heading, 1000);

    int pos = uav_state_.header.frame_id.find("/") + 1;
    printLimitedString(win, 1, 11, uav_state_.header.frame_id.substr(pos, uav_state_.header.frame_id.length()), 15);

    printLimitedString(win, 2, 11, "Hori: " + uav_state_.estimator_horizontal.name, 15);
    printLimitedString(win, 3, 11, "Vert: " + uav_state_.estimator_vertical.name, 15);
    printLimitedString(win, 4, 11, "Head: " + uav_state_.estimator_heading.name, 15);
  }
}

//}

/* mavrosStateHandler() //{ */

void Status::mavrosStateHandler(WINDOW* win, double rate, short color, int topic) {

  string tmp_string;

  switch (topic) {
    case 0:  // mavros state
      printLimitedDouble(win, 0, 9, "Mavros %5.1f Hz", rate, 1000);

      if (rate == 0) {

        printNoData(win, 0, 1);

      } else {

        if (mavros_state_.armed) {
          tmp_string = "ARMED";
        } else {
          tmp_string = "DISARMED";
          wattron(win, COLOR_PAIR(RED));
        }

        printLimitedString(win, 1, 1, "State: " + tmp_string, 15);
        wattron(win, COLOR_PAIR(color));

        if (mavros_state_.mode != "OFFBOARD") {
          wattron(win, COLOR_PAIR(RED));
        }

        printLimitedString(win, 2, 1, "Mode:  " + mavros_state_.mode, 15);
        wattron(win, COLOR_PAIR(color));
      }

      break;

    case 1:  // battery
      if (rate == 0) {

        printNoData(win, 3, 1, "Batt:  ");

      } else {

        double voltage = battery_.voltage;
        (voltage > 17.0) ? (voltage = voltage / 6) : (voltage = voltage / 4);

        if (voltage < 3.6) {
          wattron(win, COLOR_PAIR(RED));
        } else if (voltage < 3.7 && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        }

        printLimitedDouble(win, 3, 1, "Batt:  %4.2f V ", voltage, 10);
        wattron(win, COLOR_PAIR(color));

        printLimitedDouble(win, 3, 15, "%5.2f A", battery_.current, 100);
      }
      break;

    case 2:  // control manager cmd attitude
      if (rate == 0) {

        printNoData(win, 4, 1, "Thrst: ");

      } else {

        if (cmd_attitude_.thrust > 0.75) {
          wattron(win, COLOR_PAIR(RED));
        } else if (cmd_attitude_.thrust > 0.65 && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        }
        printLimitedDouble(win, 4, 1, "Thrst: %4.2f", cmd_attitude_.thrust, 1.01);
        wattron(win, COLOR_PAIR(color));

        short  tmp_color = GREEN;
        double mass_diff = fabs(cmd_attitude_.total_mass - _uav_mass_) / _uav_mass_;

        if (mass_diff > 0.3) {
          tmp_color = RED;
        } else if (mass_diff > 0.2) {
          tmp_color = YELLOW;
        }
        if (_uav_mass_ > 10.0 || cmd_attitude_.total_mass > 10.0) {

          wattron(win, COLOR_PAIR(NORMAL));
          printLimitedDouble(win, 4, 14, "%.1f/", _uav_mass_, 99.99);
          wattron(win, COLOR_PAIR(tmp_color));
          printLimitedDouble(win, 4, 19, "%.1f", cmd_attitude_.total_mass, 99.99);
          printLimitedString(win, 4, 22, "kg", 2);

        } else {

          wattron(win, COLOR_PAIR(NORMAL));
          printLimitedDouble(win, 4, 15, "%.1f/", _uav_mass_, 99.99);
          wattron(win, COLOR_PAIR(tmp_color));
          printLimitedDouble(win, 4, 19, "%.1f", cmd_attitude_.total_mass, 99.99);
          printLimitedString(win, 4, 22, "kg", 2);
        }
      }


      break;

    case 3:  // mavros global
      if (rate == 0) {

        wattron(win, COLOR_PAIR(RED));
        printLimitedString(win, 1, 18, "NO_GPS", 6);
        wattroff(win, COLOR_PAIR(RED));

      } else {

        wattron(win, COLOR_PAIR(GREEN));
        printLimitedString(win, 1, 18, "GPS_OK", 6);
        wattroff(win, COLOR_PAIR(GREEN));

        double gps_qual  = (mavros_global_.position_covariance[0] + mavros_global_.position_covariance[4] + mavros_global_.position_covariance[8]) / 3;
        short  tmp_color = RED;

        if (gps_qual < 5) {
          tmp_color = GREEN;
        } else if (gps_qual < 10) {
          tmp_color = YELLOW;
        }

        wattron(win, COLOR_PAIR(tmp_color));
        printLimitedDouble(win, 2, 17, "Q: %4.1f", gps_qual, 99.9);
        wattroff(win, COLOR_PAIR(tmp_color));
      }
      break;
  }
}

//}

/* controlManagerHandler() //{ */

void Status::controlManagerHandler(WINDOW* win, double rate, short color, int topic) {

  string controller;
  string tracker;
  controller = control_manager_.active_controller;
  tracker    = control_manager_.active_tracker;

  switch (topic) {
    case 0:  // mavros state
      printLimitedString(win, 0, 10, "Control Manager", 15);

      if (rate == 0) {

        printNoData(win, 0, 1);

        wattron(win, COLOR_PAIR(RED));
        mvwprintw(win, 1, 1, "NO_CONTROLLER");
        mvwprintw(win, 2, 1, "NO_TRACKER");
        wattron(win, COLOR_PAIR(color));

      } else {
        if (controller != "So3Controller") {
          if (controller != "MpcController") {
            wattron(win, COLOR_PAIR(RED));
          }
          printLimitedString(win, 1, 1, controller, 29);
        } else {
          mvwprintw(win, 1, 1, controller.c_str());
          wattron(win, COLOR_PAIR(NORMAL));
          mvwprintw(win, 1, 1 + controller.length(), "%s", "/");
        }

        wattron(win, COLOR_PAIR(color));

        if (tracker != "MpcTracker") {
          if (tracker == "LandoffTracker" && color != RED) {
            wattron(win, COLOR_PAIR(YELLOW));
          } else {
            wattron(win, COLOR_PAIR(RED));
          }

          printLimitedString(win, 2, 1, tracker, 29);

        } else {
          mvwprintw(win, 2, 1, tracker.c_str());
          wattron(win, COLOR_PAIR(NORMAL));
          mvwprintw(win, 2, 1 + tracker.length(), "%s", "/");
          wattron(win, COLOR_PAIR(color));
        }
      }

      if (!control_manager_.tracker_status.callbacks_enabled) {

        wattron(win, COLOR_PAIR(RED));
        mvwprintw(win, 1, 20, "NO_CB");
        wattroff(win, COLOR_PAIR(RED));
      }


      if (control_manager_.tracker_status.have_goal) {

        wattron(win, COLOR_PAIR(GREEN));
        mvwprintw(win, 2, 21, "FLY");
        wattroff(win, COLOR_PAIR(GREEN));

      } else {

        wattron(win, COLOR_PAIR(YELLOW));
        mvwprintw(win, 2, 21, "IDLE");
        wattroff(win, COLOR_PAIR(YELLOW));
      }
      break;

    case 1:  // mavros state

      if (controller == "So3Controller") {

        if (rate == 0) {
          printNoData(win, 1, 2 + controller.length());
        } else {
          printLimitedString(win, 1, 2 + controller.length(), gain_manager_.current_name, 10);
        }
      }
      break;

    case 2:  // mavros state

      if (tracker == "MpcTracker") {
        if (rate == 0) {
          printNoData(win, 2, 2 + tracker.length());
        } else {
          printLimitedString(win, 2, 2 + tracker.length(), constraint_manager_.current_name, 10);
        }
      }
      break;
  }
}

//}

/* flightTimeHandler() //{ */

void Status::flightTimeHandler(WINDOW* win) {

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  if (NullTracker_) {

    is_flying_ = false;

  } else {

    if (!is_flying_) {

      is_flying_        = true;
      last_flight_time_ = ros::Time::now();

    } else {

      int secs_passed = int((ros::Time::now() - last_flight_time_).toSec());

      if (secs_passed > 0) {

        secs_flown += secs_passed;
        last_flight_time_ = last_flight_time_ + ros::Duration(secs_passed);

        ofstream outputFile(_time_filename_);
        outputFile << secs_flown;
        outputFile.close();
      }
    }
  }

  wattron(win, A_BOLD);
  printLimitedInt(win, 0, 0, "ToF: %i", secs_flown, 1000);
  mvwprintw(win, 0, 13, " %s  %s  %s ", _uav_name_.c_str(), _uav_type_.c_str(), _nato_name_.c_str());

  int mins = secs_flown / 60;
  /* int tens_secs = ((secs_flown % 60) / 10) % 10; */
  int secs = secs_flown % 60;

  mvwprintw(win, 0, 0, "ToF: %i:%02i", mins, secs);
  wattroff(win, A_BOLD);
}

//}

/* generalInfoThread() //{ */

void Status::generalInfoThread() {

  ros::Time last_time;


  while (true) {

    double interval = (ros::Time::now() - last_time).toSec();

    if (interval >= 1.0 / double(general_info_window_rate_)) {

      std::scoped_lock lock(mutex_general_info_thread_);

      last_time = ros::Time::now();

      wclear(general_info_window_);

      wattron(general_info_window_, A_BOLD);
      wattroff(general_info_window_, COLOR_PAIR(NORMAL));
      wattroff(general_info_window_, A_STANDOUT);

      box(general_info_window_, 0, 0);

      if (_light_) {
        wattron(general_info_window_, A_STANDOUT);
      }

      printCpuLoad(general_info_window_);
      printMemLoad(general_info_window_);
      printCpuFreq(general_info_window_);
      printDiskSpace(general_info_window_);
    }

    sleep(general_info_window_rate_ / 10);
  }
}

//}

/* setupGenericCallbacks() //{ */

void Status::setupGenericCallbacks() {

  generic_topic_vec_.clear();
  generic_subscriber_vec_.clear();

  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;  // generic callback

  for (unsigned long i = 0; i < generic_topic_input_vec_.size(); i++) {

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
      topic tmp_topic(results[0], tmp_string, stoi(results[results.size() - 1]), generic_topic_window_rate_);
      generic_topic_vec_.push_back(tmp_topic);
    }
    catch (const invalid_argument& e) {
    }


    int    id = i;  // id to identify which topic called the generic callback
    string topic_name;

    if (generic_topic_vec_[i].topic_name.at(0) == '/') {

      topic_name = generic_topic_vec_[i].topic_name;

    } else {

      topic_name = "/" + _uav_name_ + "/" + generic_topic_vec_[i].topic_name;
    }

    callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { genericCallback(msg, topic_name, id); };
    ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 1, callback);

    generic_subscriber_vec_.push_back(tmp_subscriber);
  }
}

//}

/* MENU SETUP //{ */

/* setupMainMenu() //{ */

void Status::setupMainMenu() {

  service_vec_.clear();

  for (unsigned long i = 0; i < service_input_vec_.size(); i++) {

    if (NullTracker_ && (i == 0 || i == 1)) {
      continue;  // disable land and land home if we are not flying
    }

    if (!NullTracker_ && i == 2) {

      continue;  // disable takeoff if flying
    }

    std::vector<std::string> results;
    boost::split(results, service_input_vec_[i], [](char c) { return c == ' '; });  // split the input string into words and put them in results vector

    for (int j = 2; j < results.size(); j++) {
      results[1] = results[1] + " " + results[j];
    }

    string service_name;

    if (results[0].at(0) == '/') {
      service_name = results[0];

    } else {
      service_name = "/" + _uav_name_ + "/" + results[0];
    }

    service tmp_service(service_name, results[1]);
    tmp_service.service_client = nh_.serviceClient<std_srvs::Trigger>(service_name);

    service_vec_.push_back(tmp_service);
  }

  main_menu_text_.clear();

  for (unsigned long i = 0; i < service_vec_.size(); i++) {
    main_menu_text_.push_back(service_vec_[i].service_display_name);
  }

  main_menu_text_.push_back("Set Constraints");
  main_menu_text_.push_back("Set Gains");
  main_menu_text_.push_back("Set Controller");

  Menu menu(1, 32, main_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* setupGotoMenu() //{ */

void Status::setupGotoMenu() {

  goto_menu_inputs_.clear();
  goto_menu_text_.clear();
  goto_menu_text_.push_back(" X:                ");
  goto_menu_text_.push_back(" Y:                ");
  goto_menu_text_.push_back(" Z:                ");
  goto_menu_text_.push_back(" hdg:              ");
  goto_menu_text_.push_back(" " + uav_state_.header.frame_id + " ");

  Menu menu(1, 32, goto_menu_text_);
  menu_vec_.push_back(menu);


  for (int i = 0; i < 4; i++) {
    InputBox tmpbox(8, menu.getWin(), goto_double_vec_[i]);
    goto_menu_inputs_.push_back(tmpbox);
  }
}

//}

//}

/* PRINT FUNCTIONS //{ */

/* printMemLoad() //{ */

void Status::printMemLoad(WINDOW* win) {

  ifstream file("/proc/meminfo");
  string   line1, line2, line3, line4;
  getline(file, line1);
  getline(file, line2);
  getline(file, line3);
  getline(file, line4);
  file.close();

  vector<string> results;
  boost::split(results, line1, [](char c) { return c == ' '; });

  double ram_total;
  double ram_free;
  double ram_used;
  double buffers;

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        ram_total = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument& e) {
        ram_total = 0.0;
      }
      break;
    }
  }

  boost::split(results, line3, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        ram_free = double(stol(results[i])) / 1048576;
      }
      catch (const invalid_argument& e) {
        ram_free = 0.0;
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

  ram_used = ram_total - (ram_free + buffers);

  int    tmp_color = GREEN;
  double ram_ratio = ram_used / ram_total;
  if (ram_ratio > 0.8) {
    tmp_color = RED;
  } else if (ram_ratio > 0.6) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));

  printLimitedDouble(win, 2, 1, "RAM: %4.1f G", (ram_free + buffers), 100);
}

//}

/* printCpuLoad() //{ */

void Status::printCpuLoad(WINDOW* win) {

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

  long total_diff = total - last_total_;
  long idle_diff  = idle - last_idle_;

  double cpu_load = 100 * (double(total_diff - idle_diff) / double(total_diff));

  last_total_ = total;
  last_idle_  = idle;

  int tmp_color = GREEN;
  if (cpu_load > 80.0) {
    tmp_color = RED;
  } else if (cpu_load > 60.0) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));

  printLimitedDouble(win, 1, 1, "CPU: %4.1f %%", cpu_load, 99.9);
}

//}

/* printCpuFreq() //{ */

void Status::printCpuFreq(WINDOW* win) {

  ifstream file("/sys/devices/system/cpu/online");
  string   line;
  getline(file, line);
  file.close();

  vector<string> results;
  boost::split(results, line, [](char c) { return c == '-'; });

  int num_cores;

  try {
    num_cores = stoi(results[1]) + 1;
  }
  catch (const invalid_argument& e) {
    num_cores = 0;
  }

  long cpu_freq = 0;

  for (int i = 0; i < num_cores; i++) {
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

  double avg_cpu_ghz = double(cpu_freq / num_cores) / 1048576;


  wattron(win, COLOR_PAIR(GREEN));
  printLimitedDouble(win, 1, 16, "%4.2f GHz", avg_cpu_ghz, 10);
}

//}

/* printDiskSpace() //{ */

void Status::printDiskSpace(WINDOW* win) {

  boost::filesystem::space_info si = boost::filesystem::space(".");

  int gigas = round(si.available / 104857600);

  wattron(win, COLOR_PAIR(GREEN));
  if (gigas < 200 || gigas != last_gigas_) {
    wattron(win, COLOR_PAIR(YELLOW));
  }
  if (gigas < 100) {
    wattron(win, COLOR_PAIR(RED));
    printLimitedDouble(win, 2, 14, "HDD: %3.1f G", double(gigas) / 10, 10);
  } else {
    if (gigas < 1000) {
      printLimitedInt(win, 2, 14, "HDD:  %i G", gigas / 10, 1000);
    } else {
      printLimitedInt(win, 2, 14, "HDD: %i G", gigas / 10, 1000);
    }
  }
  last_gigas_ = gigas;
}

//}

/* printServiceResult() //{ */

void Status::printServiceResult(bool success, string msg) {

  if (_light_) {
    wattron(bottom_window_, A_STANDOUT);
  }

  wclear(bottom_window_);

  wattron(bottom_window_, A_BOLD);
  wattron(bottom_window_, COLOR_PAIR(GREEN));

  if (success) {

    printLimitedString(bottom_window_, 0, 0, "Service call success: " + msg, 120);

  } else {

    wattron(bottom_window_, COLOR_PAIR(RED));

    printLimitedString(bottom_window_, 0, 0, "Service call failed: " + msg, 120);

    wattroff(bottom_window_, COLOR_PAIR(RED));
  }

  bottom_window_clear_time_ = ros::Time::now();

  wattroff(bottom_window_, COLOR_PAIR(GREEN));
  wattroff(bottom_window_, A_BOLD);
}

//}

/* printLimitedInt() //{ */

void Status::printLimitedInt(WINDOW* win, int y, int x, string str_in, int num, int limit) {

  if (abs(num) > limit) {

    // if the number is larger than limit, replace it with scientific notation - 1e+01 to fit the screen
    for (unsigned long i = 0; i < str_in.length() - 2; i++) {
      if (str_in[i] == '.' && str_in[i + 2] == 'i') {
        str_in[i + 1] = '0';
        str_in[i + 2] = 'e';
        break;
      }
    }
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format, num);
}

//}

/* printLimitedDouble() //{ */

void Status::printLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit) {

  if (fabs(num) > limit) {

    // if the number is larger than limit, replace it with scientific notation - 1e+01 to fit the screen
    for (unsigned long i = 0; i < str_in.length() - 2; i++) {
      if (str_in[i] == '.' && str_in[i + 2] == 'f') {
        str_in[i + 1] = '0';
        str_in[i + 2] = 'e';
        break;
      }
    }
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format, num);
}

//}

/* printLimitedString() //{ */

void Status::printLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit) {

  if (str_in.length() > limit) {
    str_in.resize(limit);
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format);
}

//}

/* printNoData() //{ */

void Status::printNoData(WINDOW* win, int y, int x) {

  wattron(win, A_BLINK);
  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, "!NO DATA!");
  wattroff(win, COLOR_PAIR(RED));
  wattroff(win, A_BLINK);
}

void Status::printNoData(WINDOW* win, int y, int x, string text) {

  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, text.c_str());
  printNoData(win, y, x + text.length());
}

//}

/* printDebug() //{ */

void Status::printDebug(string msg) {

  printLimitedString(debug_window_, 0, 0, msg, 120);

  wrefresh(debug_window_);
}

//}

/* printHelp() //{ */

void Status::printHelp() {

  wclear(debug_window_);

  if (help_active_) {

    printLimitedString(debug_window_, 1, 0, "How to use mrs_status:", 120);
    printLimitedString(debug_window_, 2, 0, "Press the 'm' key to enter a services menu", 120);
    printLimitedString(debug_window_, 3, 0, "Press the 'g' key to set a goto reference", 120);
    printLimitedString(debug_window_, 4, 0, "Press the 'R' key to enter 'remote' mode to take direct control of the uav with your keyboad", 120);
    printLimitedString(debug_window_, 5, 0, "   In remote mode, use these keys to control the drone:", 120);
    printLimitedString(debug_window_, 6, 0, "      'w','s','a','d' to control pitch and roll ('h','j','k','l' works too)", 120);
    printLimitedString(debug_window_, 7, 0, "      'q','e'         to control heading", 120);
    printLimitedString(debug_window_, 8, 0, "      'r','f'         to control altitude", 120);
    printLimitedString(debug_window_, 10, 0, "You can also use topics provided by mrs_status to display info from your node, and to control it:", 120);
    printLimitedString(debug_window_, 12, 0, "   topic mrs_status/display_string (std_msgs::String)", 120);
    printLimitedString(debug_window_, 13, 0, "      Publish any string to this topic and it will show up in mrs_status", 120);
    printLimitedString(debug_window_, 15, 0, "   topic mrs_status/set_trigger_service (std_msgs::String)", 120);
    printLimitedString(debug_window_, 16, 0, "      If your node has a std_srvs:Trigger service, you can add it to the mrs_status services menu", 120);
    printLimitedString(debug_window_, 17, 0, "      Publish a String in this format: 'node_name/service_name display_name' and it will show in the menu", 120);
    printLimitedString(debug_window_, 19, 0, "Press 'h' to hide help", 120);

  } else {
    printLimitedString(debug_window_, 1, 0, "Press 'h' key for help", 120);
  }

  wrefresh(debug_window_);
}

//}

//}

/* CALLBACKS //{ */

/* uavStateCallback() //{ */

void Status::uavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {
  uav_state_topic_[0].counter++;
  uav_state_ = *msg;
}

//}

/* mavrosStateCallback() //{ */

void Status::mavrosStateCallback(const mavros_msgs::StateConstPtr& msg) {
  mavros_state_topic_[0].counter++;
  mavros_state_ = *msg;
}

//}

/* batteryCallback() //{ */

void Status::batteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
  mavros_state_topic_[1].counter++;
  battery_ = *msg;
}

//}

/* mavrosAttitudeCallback() //{ */

void Status::cmdAttitudeCallback(const mrs_msgs::AttitudeCommandConstPtr& msg) {
  mavros_state_topic_[2].counter++;
  cmd_attitude_ = *msg;
}

//}

/* mavrosGlobalCallback() //{ */

void Status::mavrosGlobalCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
  mavros_state_topic_[3].counter++;
  mavros_global_ = *msg;
}

//}

/* controlManagerCallback() //{ */

void Status::controlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {
  control_manager_topic_[0].counter++;
  control_manager_                                                = *msg;
  control_manager_.active_tracker == "NullTracker" ? NullTracker_ = true : NullTracker_ = false;
}

//}

/* gainManagerCallback() //{ */

void Status::gainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg) {
  control_manager_topic_[1].counter++;
  gain_manager_ = *msg;
}

//}

/* constraintManagerCallback() //{ */

void Status::constraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {
  control_manager_topic_[2].counter++;
  constraint_manager_ = *msg;
}

//}

/* setServiceCallback() //{ */

void Status::setServiceCallback(const std_msgs::String& msg) {

  if (std::find(service_input_vec_.begin(), service_input_vec_.end(), msg.data) == service_input_vec_.end()) {
    service_input_vec_.push_back(msg.data);
  }
}
//}

/* tfStaticCallback() //{ */

void Status::tfStaticCallback(const tf2_msgs::TFMessage& msg) {

  for (unsigned long i = 0; i < msg.transforms.size(); i++) {

    std::string tmp        = msg.transforms[i].child_frame_id;
    std::size_t pos        = tmp.find("/");        // find the / in uav1/something
    std::string uav_name   = tmp.substr(0, pos);   // cut out the uav name, so we can discard tfs from other drones (mostly for simulation)
    std::string frame_name = tmp.substr(pos + 1);  // cut off the uav1/ from the tf_static name

    for (unsigned long j = 0; j < tf_static_list_compare_.size(); j++) {
      if (tf_static_list_compare_[j] == frame_name && uav_name == _uav_name_) {
        if (std::find(generic_topic_input_vec_.begin(), generic_topic_input_vec_.end(), tf_static_list_add_[j]) == generic_topic_input_vec_.end())
          generic_topic_input_vec_.push_back(tf_static_list_add_[j]);
      }
    }
  }

  setupGenericCallbacks();
}
//}

/* stringCallback() //{ */

void Status::stringCallback(const ros::MessageEvent<std_msgs::String const>& event) {

  std::string pub_name = event.getPublisherName();
  std::string msg_str  = event.getMessage()->data;

  bool contains = false;

  for (unsigned long i = 0; i < string_info_vec_.size(); i++) {
    if (string_info_vec_[i].publisher_name == pub_name) {
      contains                           = true;
      string_info_vec_[i].display_string = msg_str;
      string_info_vec_[i].last_time      = ros::Time::now();
      break;
    }
  }

  if (!contains) {
    string_info tmp(pub_name, msg_str);
    string_info_vec_.push_back(tmp);
  }
}
//}

/* genericCallback() //{ */

void Status::genericCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name, const int id) {
  generic_topic_vec_[id].counter++;
}

//}

//}

/* callTerminal //{ */

std::string Status::callTerminal(const char* cmd) {
  std::array<char, 128>                    buffer;
  std::string                              result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    ROS_ERROR("[Status]: Exception in callTerminal");
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

//}

/* main() //{ */

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrs_status");

  initscr();
  start_color();
  cbreak();
  noecho();
  clear();
  nodelay(stdscr, true);
  keypad(stdscr, true);
  timeout(0);
  curs_set(0);  // disable cursor
  set_escdelay(0);
  use_default_colors();

  init_pair(NORMAL, COLOR_WHITE, BACKGROUND_DEFAULT);
  init_pair(FIELD, COLOR_WHITE, 235);
  init_pair(RED, COLOR_NICE_RED, BACKGROUND_DEFAULT);
  init_pair(YELLOW, COLOR_NICE_YELLOW, BACKGROUND_DEFAULT);
  init_pair(GREEN, COLOR_NICE_GREEN, BACKGROUND_DEFAULT);
  init_pair(BLUE, COLOR_NICE_BLUE, BACKGROUND_DEFAULT);

  attron(A_BOLD);

  Status status;

  if (status._colorscheme_ == "COLORSCHEME_LIGHT") {
    init_pair(NORMAL, COLOR_BLACK, BACKGROUND_DEFAULT);
    init_pair(FIELD, COLOR_WHITE, 237);
    init_pair(GREEN, COLOR_DARK_GREEN, BACKGROUND_DEFAULT);
    init_pair(BLUE, COLOR_DARK_BLUE, BACKGROUND_DEFAULT);
    init_pair(YELLOW, COLOR_DARK_YELLOW, BACKGROUND_DEFAULT);
    status._light_ = true;
  }

  if (status._rainbow_) {

    /* DEBUG COLOR RAINBOW //{ */
    int k = 0;

    for (int j = 0; j < 256; j++) {
      init_pair(j, COLOR_WHITE, j);
    }

    for (int j = 0; j < 256; j++) {

      attron(COLOR_PAIR(j));
      mvwprintw(stdscr, k + 10, (3 * j + 1) - k * 60 * 3, "%i", j);
      k = int(j / 60);
      attroff(COLOR_PAIR(j));
    }
    refresh();
    while (1) {
    }

    //}
  }

  while (ros::ok()) {

    ros::spin();
    return 0;
  }
}

//

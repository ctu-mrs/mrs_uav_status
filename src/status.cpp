/* includes //{ */

#include <menu.h>

/* #include <status_window.h> */
#include <mrs_msgs/msg/node_cpu_load.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/gimbal_state.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>

#include <input_box.h>
#include <commons.h>
#include <iostream>
#include <fstream>

#include <mrs_lib/node.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>

#include <boost/filesystem.hpp>

using namespace std;

using radians = mrs_lib::geometry::radians;

//}

/* typedefs //{ */

typedef enum
{
  STANDARD,
  REMOTE,
  GIMBAL,
  MAIN_MENU,
  GOTO_MENU,
  DISPLAY_MENU,
} status_state;

//}

/* defines //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_status
{

/* class Status //{ */


class Status : public mrs_lib::Node {

public:
  Status();

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  void initialize();

public:
  std::string _colorscheme_;
  std::string _pwd_;
  std::string _display_config_filename_;

  bool _light_ = false;

  std::mutex mutex_status_msg_;

  mrs_msgs::msg::UavStatus uav_status_;

  // | ------------------------- Timers ------------------------- |

  std::shared_ptr<TimerType> timer_status_fast_;
  std::shared_ptr<TimerType> timer_status_slow_;
  std::shared_ptr<TimerType> timer_resize_;

  void timerStatusFast();
  void timerStatusSlow();
  void timerResize();

  // | --------------------- Print routines --------------------- |

  void printLimitedInt(WINDOW *win, int y, int x, string str_in, int num, int limit);
  void printLimitedDouble(WINDOW *win, int y, int x, string str_in, double num, double limit);
  void printLimitedString(WINDOW *win, int y, int x, string str_in, unsigned long limit);
  void printCompressedLimitedString(WINDOW *win, int y, int x, string str_in, unsigned long limit);
  void printServiceResult(bool success, string msg);
  void printError(string msg);
  void printDebug(string msg);
  void printHelp();
  void printTmuxDump();
  void printBox(WINDOW *win);

  void printNoData(WINDOW *win, int y, int x);
  void printNoData(WINDOW *win, int y, int x, string text);

  // | ------------------------- Windows ------------------------ |

  void setupWindows();

  void uavStateHandler(WINDOW *win);
  void controlManagerHandler(WINDOW *win);
  void hwApiStateHander(WINDOW *win);
  void genericTopicHandler(WINDOW *win);
  void nodeStatsHandler(WINDOW *win);
  void generalInfoHandeler(WINDOW *win);
  void stringHandler(WINDOW *);

  double general_info_window_rate_  = 1;
  double generic_topic_window_rate_ = 1;

  bool increment_counter_         = false;
  int  estimator_display_counter_ = 0;

  int    _service_num_calls_ = 20;
  double _service_delay_     = 0.1;

  void printCpuLoad(WINDOW *win);
  void printCpuTemp(WINDOW *win);
  void printCpuFreq(WINDOW *win);
  void printMemLoad(WINDOW *win);
  void printDiskSpace(WINDOW *win);

  long last_idle_  = 0;
  long last_total_ = 0;
  long last_gigas_ = 0;

  // | ------------------------- Subscribers ------------------------ |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>      sh_uav_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatusShort> sh_uav_status_short_;

  // | ------------------------- Publishers ------------------------ |

  mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState> ph_gimbal_state_;

  // | ------------------------- Callbacks ------------------------- |

  void callbackUavStatus(const mrs_msgs::msg::UavStatus::ConstSharedPtr msg);
  void callbackUavStatusShort(const mrs_msgs::msg::UavStatusShort::ConstSharedPtr msg);

  // Custom windows
  WINDOW *uav_state_window_;
  WINDOW *control_manager_window_;
  WINDOW *hw_api_state_window_;

  /* vector<string_info> string_info_vec_; */
  vector<TopicInfo> string_topic_;

  // Vanilla windows
  WINDOW *top_bar_window_;
  WINDOW *bottom_window_;
  WINDOW *generic_topic_window_;
  WINDOW *node_stats_window_;
  WINDOW *general_info_window_;
  WINDOW *debug_window_;
  WINDOW *sub_tmux_window_1_;
  WINDOW *sub_tmux_window_2_;
  WINDOW *string_window_;

  rclcpp::Time bottom_window_clear_time_;
  rclcpp::Time last_time_got_data_;
  rclcpp::Time last_time_got_short_data_;

  unsigned long line_in_upper_menu_;

  bool help_active_ = false;

  /* StatusWindow* generic_topic_window_; */

  // | ---------------------- Misc routines --------------------- |

  bool updateTermSize();

  void prefillUavStatus();

  void topLineHandler(WINDOW *win);

  void remoteHandler(int key, WINDOW *win);

  void gimbalHandler(int key, WINDOW *win);

  void remoteModeFly(const mrs_msgs::msg::Reference &ref_in);

  void setupColors(bool active);

  std::string callTerminal(const char *cmd);

  mrs_lib::Profiler profiler_;
  bool              _colorblind_mode_  = false;
  bool              _profiler_enabled_ = false;


  // | ---------------------- Menu routines --------------------- |

  void setupMainMenu();
  bool mainMenuHandler(int key_in);

  void setupGotoMenu();
  bool gotoMenuHandler(int key_in);

  void setupDisplayMenu();
  void setupDisplayText();

  bool displayMenuHandler(int key_in);

  // | --------------------- Service Clients -------------------- |


  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>    service_goto_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv> service_trajectory_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                 service_set_constraints_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                 service_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                 service_set_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                 service_set_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                 service_set_estimator_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                service_hover_;

  // | -------------------- UAV configuration ------------------- |

  string _uav_type_;
  /* string _run_type_; */
  /* double _uav_mass_; */
  /* string _sensors_; */
  /* bool   _pixgarm_; */
  string _turbo_remote_constraints_;

  // | ------------------ Data storage, inputs ------------------ |

  /* vector<TopicInfo>       generic_topic_vec_; */
  /* vector<string>          generic_topic_input_vec_; */
  /* vector<ros::Subscriber> generic_subscriber_vec_; */

  vector<Menu> menu_vec_;
  vector<Menu> submenu_vec_;

  vector<service> service_vec_;
  vector<string>  service_input_vec_;
  vector<string>  main_menu_text_;
  vector<string>  display_menu_text_;
  vector<string>  constraints_text_;
  vector<string>  gains_text_;
  vector<string>  controllers_text_;
  vector<string>  trackers_text_;
  vector<string>  odometry_lat_sources_text_;
  vector<string>  odometry_alt_sources_text_;
  vector<string>  odometry_hdg_sources_text_;

  vector<double>   goto_double_vec_;
  vector<string>   goto_menu_text_;
  vector<InputBox> goto_menu_inputs_;

  string old_constraints;

  mrs_msgs::msg::GimbalState gimbal_command;
  const uint16_t             gimbal_max = 2000;
  const uint16_t             gimbal_min = 1000;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | -------------------- Switches, states -------------------- |

  bool remote_hover_    = false;
  bool turbo_remote_    = false;
  bool remote_global_   = false;
  bool have_data_       = false;
  bool have_short_data_ = false;

  bool avoiding_collision_          = false;
  bool automatic_start_can_takeoff_ = false;
  bool null_tracker_                = false;
  bool is_flying_                   = false;

  status_state state = STANDARD;
  int          cols_, lines_;

  std::atomic<bool> initialized_ = false;

  bool             mini_ = false;
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
};

//}

/* Status() //{ */

Status::Status() : Node("mrs_status_menu") {

  initscr();
  start_color();
  cbreak();
  noecho();
  clear();
  nodelay(stdscr, true);
  keypad(stdscr, true);
  timeout(0);
  curs_set(0); // disable cursor
  set_escdelay(0);
  use_default_colors();

  attron(A_BOLD);

  initialize();
}

//}

/* initialize() //{ */

void Status::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | ---------------------- Param loader ---------------------- |

  bottom_window_clear_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_time_got_data_       = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_time_got_short_data_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  prefillUavStatus();

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

  param_loader.loadParam("pwd", _pwd_);

  param_loader.loadParam("colorscheme", _colorscheme_);

  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", _turbo_remote_constraints_);

  param_loader.loadParam("mrs_uav_status/colorblind_mode", _colorblind_mode_);
  param_loader.loadParam("mrs_uav_status/enable_profiler", _profiler_enabled_);

  param_loader.loadParam("mrs_uav_status/start_minimized", mini_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  } else {
    RCLCPP_INFO(node_->get_logger(), "All params loaded!");
  }

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&Status::timerStatusFast, this);

    timer_status_fast_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(20.0, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Status::timerStatusSlow, this);

    timer_status_slow_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(1.0, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Status::timerResize, this);

    timer_resize_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(1.0, clock_), callback_fcn);
  }

  // | ------------------------ Subscribers ------------------------ |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_uav_status_       = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>(shopts, "~/uav_status_in", &Status::callbackUavStatus, this);
  sh_uav_status_short_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatusShort>(shopts, "~/uav_status_short_in", &Status::callbackUavStatusShort, this);

  // | ------------------------ Publishers ------------------------ |

  ph_gimbal_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState>(node_, "~/gimbal_command_out");

  // | --------------------- service clients -------------------- |

  service_goto_reference_       = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/reference_out", cbkgrp_sc_);
  service_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>(node_, "~/trajectory_reference_out", cbkgrp_sc_);
  service_set_constraints_      = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_constraints_out", cbkgrp_sc_);
  service_set_gains_            = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_gains_out", cbkgrp_sc_);
  service_set_controller_       = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_controller_out", cbkgrp_sc_);
  service_set_tracker_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_tracker_out", cbkgrp_sc_);
  service_set_estimator_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_estimator_out", cbkgrp_sc_);
  service_hover_                = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out", cbkgrp_sc_);

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  // Loads the default GoTo value
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(2.0);
  goto_double_vec_.push_back(1.57);

  // Loads the default menu options
  service_input_vec_.push_back("uav_manager/land Land");
  service_input_vec_.push_back("uav_manager/land_home Land Home");
  service_input_vec_.push_back("uav_manager/takeoff Takeoff");

  // --------------------------------------------------------------
  // |            Window creation and topic association           |
  // --------------------------------------------------------------

  updateTermSize();
  setupWindows();

  _display_config_filename_ = _pwd_ + "/.mrs_status_display_config~";

  if (boost::filesystem::exists(_display_config_filename_)) {

    selected_tmux_window_.clear();

    std::ifstream file(_display_config_filename_);

    std::string line;

    for (int i = 0; i < MAX_SELECTED_TMUX_WINDOWS; i++) {

      getline(file, line);

      try {
        selected_tmux_window_.push_back(stoi(line));
      }
      catch (const invalid_argument &e) {
      }
    }

    file.close();

    setupDisplayText();
  }

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* setupWindows() //{ */

void Status::setupWindows() {

  std::string command = "tmux display-message -p '#S'";
  session_name_       = callTerminal(command.c_str());
  session_name_.erase(std::remove(session_name_.begin(), session_name_.end(), '\n'), session_name_.end());

  command                           = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string              response = callTerminal(command.c_str());
  std::vector<std::string> results;
  boost::split(results, response, [](char c) { return c == 'x'; });

  /* int cols, lines; */

  /* try { */
  /*   cols  = stoi(results[0]); */
  /*   lines = stoi(results[1]); */
  /* } */

  /* catch (const invalid_argument& e) { */
  /*   cols  = 0; */
  /*   lines = 0; */
  /* } */

  if (mini_) {

    control_manager_window_ = newwin(4, 9, 1, 1);
    uav_state_window_       = newwin(6, 9, 5, 1);
    top_bar_window_         = newwin(1, 140, 0, 1);
    general_info_window_    = newwin(4, 9, 1, 10);
    hw_api_state_window_    = newwin(6, 9, 5, 10);
    debug_window_           = newwin(lines_ - 15, cols_ - 1, 13, 1);
    generic_topic_window_   = newwin(10, 9, 1, 19);
    string_window_          = newwin(10, 15, 1, 28);
    bottom_window_          = newwin(1, 120, 11, 1);

  } else {

    uav_state_window_       = newwin(7, 26, 5, 1);
    control_manager_window_ = newwin(4, 26, 1, 1);
    hw_api_state_window_    = newwin(7, 25, 5, 27);
    general_info_window_    = newwin(4, 25, 1, 27);
    top_bar_window_         = newwin(1, 140, 0, 1);
    bottom_window_          = newwin(1, 120, 12, 1);
    debug_window_           = newwin(lines_ - 15, cols_ - 1, 13, 1);
    int half_lines          = (lines_ - 18) / 2;
    sub_tmux_window_1_      = derwin(debug_window_, half_lines, cols_ - 3, 1, 1);
    sub_tmux_window_2_      = derwin(debug_window_, half_lines, cols_ - 3, half_lines + 2, 1);

    generic_topic_window_ = newwin(11, 25, 1, 52);
    string_window_        = newwin(11, 32, 1, 77);
    node_stats_window_    = newwin(11, 50, 1, 109);
  }

  clear();
  setupColors(have_data_);
}

//}

/* timerResize() //{ */

void Status::timerResize() {

  if (!initialized_) {
    return;
  }

  if (updateTermSize()) {

    if (cols_ > 30) {
      resize_term(lines_, cols_);
      setupWindows();
    }
  }
}

//}

/* updateTermSize() //{ */

bool Status::updateTermSize() {

  bool changed = false;

  std::string command  = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string response = callTerminal(command.c_str());

  std::vector<std::string> results;

  boost::split(results, response, [](char c) { return c == 'x'; });

  int cols, lines;

  try {
    cols  = stoi(results[0]);
    lines = stoi(results[1]);
  }

  catch (const invalid_argument &e) {
    cols  = 0;
    lines = 0;
  }

  if (cols_ != cols || lines_ != lines) {
    lines_  = lines;
    cols_   = cols;
    changed = true;
  }

  return (changed);
}


//}

/* timerStatusFast() //{ */

void Status::timerStatusFast() {

  if (!initialized_) {
    return;
  }

  topLineHandler(top_bar_window_);

  if (!mini_) {
    if (!selected_tmux_window_.empty()) {
      printTmuxDump();
    } else {
      printHelp();
    }
  }

  if (estimator_display_counter_ == 3) {
    estimator_display_counter_ = 0;
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("uavStateHandler");
    uavStateHandler(uav_state_window_);
  }

  if ((clock_->now() - bottom_window_clear_time_).seconds() > 3.0) {
    werase(bottom_window_);
  }

  int  key_in = getch();
  bool is_flying_normally_;

  switch (state) {

    /* STANDARD //{ */

  case STANDARD: {

    switch (key_in) {

      /* R //{ */

    case 'R': {

      {
        std::scoped_lock lock(mutex_status_msg_);
        is_flying_normally_ = uav_status_.flying_normally;
      }

      if (is_flying_normally_) {
        remote_hover_ = false;
        state         = REMOTE;
      }

      break;
    }

      //}

      /* G //{ */

    case 'G': {

      gimbal_command.fpv_mode    = true;
      gimbal_command.is_on       = true;
      gimbal_command.gimbal_pan  = 1500;
      gimbal_command.gimbal_tilt = 1500;
      state                      = GIMBAL;
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

    case 'M':
      mini_ = !mini_;
      setupWindows();
      timerStatusFast();
      timerStatusSlow();
      break;

    case 'D':
      setupDisplayMenu();
      state = DISPLAY_MENU;
      break;

    default:
      flushinp();
      break;
    }
    }

    //}

    break;
  }

    //}

    /* REMOTE //{ */

  case REMOTE: {

    flushinp();
    remoteHandler(key_in, top_bar_window_);

    if (key_in == 'R' || key_in == KEY_ESC) {

      if (turbo_remote_) {

        turbo_remote_ = false;

        auto request = std::make_shared<mrs_msgs::srv::String::Request>();

        request->value = old_constraints;

        auto response = service_set_constraints_.callSync(request);

        if (response) {
          printServiceResult(response.value()->success, response.value()->message);
        } else {
          printServiceResult(false, "service could not be called");
        }
      }

      state = STANDARD;
    }

    break;
  }

    //}

    /* GIMBAL //{ */

  case GIMBAL: {

    flushinp();

    gimbalHandler(key_in, top_bar_window_);

    if (key_in == 'G' || key_in == KEY_ESC) {
      state = STANDARD;
    }

    break;
  }

    //}

    /* MAIN_MENU //{ */

  case MAIN_MENU: {

    flushinp();

    if (mainMenuHandler(key_in)) {

      menu_vec_.clear();
      submenu_vec_.clear();

      wnoutrefresh(debug_window_);
      wnoutrefresh(bottom_window_);

      state = STANDARD;
    }

    break;
  }

    //}

    /* GOTO_MENU //{ */

  case GOTO_MENU: {

    flushinp();

    if (gotoMenuHandler(key_in)) {
      menu_vec_.clear();
      submenu_vec_.clear();
      state = STANDARD;
    }

    break;
  }

    //}

    /* DISPLAY_MENU //{ */

  case DISPLAY_MENU: {

    flushinp();

    if (displayMenuHandler(key_in)) {
      menu_vec_.clear();
      submenu_vec_.clear();
      state = STANDARD;
    }

    break;
  }

    //}
  }

  /* if (state == STANDARD) { */
  /*   printDebug("standard"); */
  /* } else if (state == REMOTE) { */
  /*   printDebug("remote"); */
  /* } else { */
  /*   printDebug("something else"); */
  /* } */

  if (state != MAIN_MENU && state != GOTO_MENU && state != DISPLAY_MENU) {
    wnoutrefresh(bottom_window_);
    /* wrefresh(bottom_window_); */
  }

  wnoutrefresh(top_bar_window_);
  /* wrefresh(top_bar_window_); */

  /* refresh(); */

  doupdate();

  /* fflush(stdout); */
}

//}

/* timerStatusSlow() //{ */

void Status::timerStatusSlow() {

  if (!initialized_) {
    return;
  }

  increment_counter_ = !increment_counter_;
  estimator_display_counter_ += int(increment_counter_);

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("hwApiStateHander");
    hwApiStateHander(hw_api_state_window_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("controlManagerHandler");
    controlManagerHandler(control_manager_window_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("genericTopicHandler");
    genericTopicHandler(generic_topic_window_);
  }

  if (!mini_) {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("nodeStatsHandler");
    nodeStatsHandler(node_stats_window_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("stringHandler");
    stringHandler(string_window_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("generalInfoHandler");
    generalInfoHandeler(general_info_window_);
  }
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

    case 0:

      // TRIGGER CONFIRMATION
      ret = submenu_vec_[0].iterate(key_in, true);

      if (ret.has_value()) {

        int line = get<0>(ret.value());
        int key  = get<1>(ret.value());

        if (line == 666 && key == 666) {

          submenu_vec_.clear();
          return false;

        } else if (key == KEY_ENT) {

          if (line == 1) {

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

            auto response = service_vec_[line_in_upper_menu_].service_client.callSync(request);

            if (response) {
              printServiceResult(response.value()->success, response.value()->message);
            } else {
              printServiceResult(false, "service could not be called");
            }

            submenu_vec_.clear();

            return true;

          } else {

            submenu_vec_.clear();
            return false;
          }
        }
      }
      break;

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

          auto request = std::make_shared<mrs_msgs::srv::String::Request>();

          request->value = constraints_text_[line];

          auto response = service_set_constraints_.callSync(request);

          if (response) {
            printServiceResult(response.value()->success, response.value()->message);
          } else {
            printServiceResult(false, "service could not be called");
          }

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

          auto request = std::make_shared<mrs_msgs::srv::String::Request>();

          request->value = gains_text_[line];

          auto response = service_set_gains_.callSync(request);

          if (response) {
            printServiceResult(response.value()->success, response.value()->message);
          } else {
            printServiceResult(false, "service could not be called");
          }

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

          auto request = std::make_shared<mrs_msgs::srv::String::Request>();

          request->value = controllers_text_[line];

          auto response = service_set_controller_.callSync(request);

          if (response) {
            printServiceResult(response.value()->success, response.value()->message);
          } else {
            printServiceResult(false, "service could not be called");
          }

          submenu_vec_.clear();
          return true;
        }
      }
      break;

    case 4:
      // TRACKERS
      ret = submenu_vec_[0].iterate(trackers_text_, key_in, true);

      if (ret.has_value()) {

        int line = get<0>(ret.value());
        int key  = get<1>(ret.value());

        if (line == 666 && key == 666) {

          submenu_vec_.clear();
          return false;

        } else if (key == KEY_ENT) {

          auto request = std::make_shared<mrs_msgs::srv::String::Request>();

          request->value = trackers_text_[line];

          auto response = service_set_tracker_.callSync(request);

          if (response) {
            printServiceResult(response.value()->success, response.value()->message);
          } else {
            printServiceResult(false, "service could not be called");
          }

          submenu_vec_.clear();
          return true;
        }
      }
      break;

    case 5:
      // Odometry source
      ret = submenu_vec_[0].iterate(odometry_lat_sources_text_, key_in, true);

      if (ret.has_value()) {

        int line = get<0>(ret.value());
        int key  = get<1>(ret.value());

        if (line == 666 && key == 666) {

          submenu_vec_.clear();
          return false;

        } else if (key == KEY_ENT) {

          auto request = std::make_shared<mrs_msgs::srv::String::Request>();

          request->value = odometry_lat_sources_text_[line];

          auto response = service_set_estimator_.callSync(request);

          if (response) {
            printServiceResult(response.value()->success, response.value()->message);
          } else {
            printServiceResult(false, "service could not be called");
          }

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

      unsigned long line = get<0>(ret.value());
      int           key  = get<1>(ret.value());

      if (line == 666 && key == 666) {

        menu_vec_.clear();
        return true;

      } else if (key == KEY_ENT) {

        if (line < service_vec_.size()) {

          int                  x;
          int                  y;
          [[maybe_unused]] int rows;
          int                  cols;

          line_in_upper_menu_ = line;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          std::vector<std::string> confirm_text;
          confirm_text.push_back("CANCEL");
          confirm_text.push_back(main_menu_text_[line]);
          Menu menu(x, 31 + cols, confirm_text, 0);
          submenu_vec_.push_back(menu);

        } else if (line == main_menu_text_.size() - 5) {
          // SET CONSTRAINTS

          {
            std::scoped_lock lock(mutex_status_msg_);
            constraints_text_ = uav_status_.constraints;
          }

          if (!constraints_text_.empty()) {

            int                  x;
            int                  y;
            [[maybe_unused]] int rows;
            int                  cols;

            getyx(menu_vec_[0].getWin(), x, y);
            getmaxyx(menu_vec_[0].getWin(), rows, cols);

            Menu menu(x, 31 + cols, constraints_text_, 1);
            submenu_vec_.push_back(menu);
          }

        } else if (line == main_menu_text_.size() - 4) {
          // SET GAINS

          {
            std::scoped_lock lock(mutex_status_msg_);
            gains_text_ = uav_status_.gains;
          }

          if (!gains_text_.empty()) {

            int                  x;
            int                  y;
            [[maybe_unused]] int rows;
            int                  cols;

            getyx(menu_vec_[0].getWin(), x, y);
            getmaxyx(menu_vec_[0].getWin(), rows, cols);

            Menu menu(x, 31 + cols, gains_text_, 2);
            submenu_vec_.push_back(menu);
          }

        } else if (line == main_menu_text_.size() - 3) {
          // SET CONTROLLER

          {
            std::scoped_lock lock(mutex_status_msg_);
            controllers_text_ = uav_status_.controllers;
          }

          if (!controllers_text_.empty()) {

            int                  x;
            int                  y;
            [[maybe_unused]] int rows;
            int                  cols;

            getyx(menu_vec_[0].getWin(), x, y);
            getmaxyx(menu_vec_[0].getWin(), rows, cols);

            Menu menu(x, 31 + cols, controllers_text_, 3);
            submenu_vec_.push_back(menu);
          }

        } else if (line == main_menu_text_.size() - 2) {
          // SET TRACKER

          {
            std::scoped_lock lock(mutex_status_msg_);
            trackers_text_ = uav_status_.trackers;
          }

          if (!trackers_text_.empty()) {

            int                  x;
            int                  y;
            [[maybe_unused]] int rows;
            int                  cols;

            getyx(menu_vec_[0].getWin(), x, y);
            getmaxyx(menu_vec_[0].getWin(), rows, cols);

            Menu menu(x, 31 + cols, trackers_text_, 4);
            submenu_vec_.push_back(menu);
          }

        } else if (line == main_menu_text_.size() - 1) {
          // SET ODOMETRY SOURCE

          {
            std::scoped_lock lock(mutex_status_msg_);
            odometry_lat_sources_text_ = uav_status_.odom_estimators;
          }

          if (!odometry_lat_sources_text_.empty()) {

            int                  x;
            int                  y;
            [[maybe_unused]] int rows;
            int                  cols;

            getyx(menu_vec_[0].getWin(), x, y);
            getmaxyx(menu_vec_[0].getWin(), rows, cols);

            Menu menu(x, 31 + cols, odometry_lat_sources_text_, 5);
            submenu_vec_.push_back(menu);
          }

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
    size_t line = get<0>(ret.value());
    int    key  = get<1>(ret.value());

    if (line == 666 && key == 666) {

      menu_vec_.clear();
      return true;

    } else if (key == KEY_ENT) {

      goto_double_vec_[0] = goto_menu_inputs_[0].getDouble();
      goto_double_vec_[1] = goto_menu_inputs_[1].getDouble();
      goto_double_vec_[2] = goto_menu_inputs_[2].getDouble();
      goto_double_vec_[3] = goto_menu_inputs_[3].getDouble();

      auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

      request->reference.position.x = goto_double_vec_[0];
      request->reference.position.y = goto_double_vec_[1];
      request->reference.position.z = goto_double_vec_[2];
      request->reference.heading    = goto_double_vec_[3];

      {
        std::scoped_lock lock(mutex_status_msg_);
        request->header.frame_id = uav_status_.odom_frame;
      }

      auto response = service_goto_reference_.callSync(request);

      if (response) {
        printServiceResult(response.value()->success, response.value()->message);
      } else {
        printServiceResult(false, "service could not be called");
      }

      menu_vec_.clear();

      return true;

    } else if (line < goto_menu_inputs_.size()) {

      goto_menu_inputs_[line].Process(key);
    }
  }


  for (size_t i = 0; i < goto_menu_inputs_.size(); i++) {
    if (int(i) == menu_vec_[0].getLine()) {
      goto_menu_inputs_[i].Print(i + 1, true);
    } else {
      goto_menu_inputs_[i].Print(i + 1, false);
    }
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return false;
}

//}

/* displayMenuHandler() //{ */

bool Status::displayMenuHandler(int key_in) {

  optional<tuple<int, int>> ret = menu_vec_[0].iterate(display_menu_text_, key_in, false);

  if (ret.has_value()) {
    size_t line = get<0>(ret.value());
    int    key  = get<1>(ret.value());

    if (line == 666 && key == 666) {

      menu_vec_.clear();
      return true;
    }

    else if (key == KEY_ENT) {

      auto it = std::find(selected_tmux_window_.begin(), selected_tmux_window_.end(), line);

      if (it != selected_tmux_window_.end()) {
        display_menu_text_[line][1] = ' ';
        selected_tmux_window_.erase(it);
      } else if (int(selected_tmux_window_.size()) < MAX_SELECTED_TMUX_WINDOWS) {
        display_menu_text_[line][1] = '*';
        selected_tmux_window_.push_back(line);
      }

      std::ofstream outputFile(_display_config_filename_, std::ofstream::out | std::ofstream::trunc);

      for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
        outputFile << selected_tmux_window_[i] << '\n';
      }
      outputFile.close();
    }
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return false;
}

//}

/* remoteHandler() //{ */

void Status::remoteHandler(int key, WINDOW *win) {

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(RED));
  if (mini_) {
    mvwprintw(win, 0, 33, "REM");
  } else {
    mvwprintw(win, 0, 55, "REMOTE MODE");
  }

  if (remote_global_) {
    if (mini_) {
      mvwprintw(win, 0, 37, "G");
    } else {
      mvwprintw(win, 0, 75, "GLOBAL MODE");
    }
  } else {
    if (mini_) {
      mvwprintw(win, 0, 37, "L");
    } else {
      mvwprintw(win, 0, 75, "LOCAL MODE");
    }
  }

  if (turbo_remote_) {
    wattron(win, A_BLINK);
    if (mini_) {
      mvwprintw(win, 0, 39, "!T!");
    } else {
      mvwprintw(win, 0, 67, "!TURBO!");
    }
    wattroff(win, A_BLINK);
  }

  wattroff(win, COLOR_PAIR(RED));

  mrs_msgs::msg::Reference       reference;
  mrs_msgs::srv::String::Request string_service;

  reference.position.x = 0.0;
  reference.position.y = 0.0;
  reference.position.z = 0.0;
  reference.heading    = 0.0;

  switch (key) {

  case 'w':
  case 'k':
  case KEY_UP:
    reference.position.x = 2.0;

    if (turbo_remote_) {
      reference.position.x = 5.0;
    }
    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 's':
  case 'j':
  case KEY_DOWN:
    reference.position.x = -2.0;

    if (turbo_remote_) {
      reference.position.x = -5.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'a':
  case 'h':
  case KEY_LEFT:
    reference.position.y = 2.0;

    if (turbo_remote_) {
      reference.position.y = 5.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'd':
  case 'l':
  case KEY_RIGHT:
    reference.position.y = -2.0;

    if (turbo_remote_) {
      reference.position.y = -5.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'r':
    reference.position.z = 1.0;

    if (turbo_remote_) {
      reference.position.z = 2.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'f':
    reference.position.z = -1.0;

    if (turbo_remote_) {
      reference.position.z = -2.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'q':
    reference.heading = 0.5;

    if (turbo_remote_) {
      reference.heading = 1.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'e':
    reference.heading = -0.5;

    if (turbo_remote_) {
      reference.heading = -1.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'T':

    bool is_flying_normally_;

    {
      std::scoped_lock lock(mutex_status_msg_);
      is_flying_normally_ = uav_status_.flying_normally;
    }

    if (is_flying_normally_) {

      if (turbo_remote_) {

        turbo_remote_ = false;

        auto request = std::make_shared<mrs_msgs::srv::String::Request>();

        request->value = old_constraints;

        auto response = service_set_constraints_.callSync(request);

        if (response) {
          printServiceResult(response.value()->success, response.value()->message);
        } else {
          printServiceResult(false, "service could not be called");
        }

      } else {

        turbo_remote_ = true;

        {
          std::scoped_lock lock(mutex_status_msg_);
          old_constraints = uav_status_.constraints[0];
        }

        auto request = std::make_shared<mrs_msgs::srv::String::Request>();

        request->value = _turbo_remote_constraints_;

        auto response = service_set_constraints_.callSync(request);

        if (response) {
          printServiceResult(response.value()->success, response.value()->message);
        } else {
          printServiceResult(false, "service could not be called");
        }
      }
    }

    break;

  case 'G': {

    {
      std::scoped_lock lock(mutex_status_msg_);
      is_flying_normally_ = uav_status_.flying_normally;
    }

    if (is_flying_normally_) {
      remote_global_ = !remote_global_;
    }

    break;
  }

  default: {

    if (remote_hover_) {

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

      service_hover_.callSync(request);

      remote_hover_ = false;
    }

    break;
  }
  }

  wattroff(win, A_BOLD);
}

//}

/* gimbalHandler() //{ */

void Status::gimbalHandler(int key, WINDOW *win) {

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, 0, 43, "GIMBAL      MODE IS ACTIVE");

  if (gimbal_command.fpv_mode) {
    mvwprintw(win, 0, 50, "FPV");
  } else {
    mvwprintw(win, 0, 50, "P-T");
  }

  wattroff(win, COLOR_PAIR(RED));

  const uint16_t gimbal_max       = 2000;
  const uint16_t gimbal_min       = 1000;
  uint16_t       gimbal_increment = 10;

  switch (key) {

  case 'w':
  case 'k':
  case KEY_UP:
    gimbal_command.gimbal_tilt -= gimbal_increment;
    break;

  case 's':
  case 'j':
  case KEY_DOWN:
    gimbal_command.gimbal_tilt += gimbal_increment;
    break;

  case 'a':
  case 'h':
  case KEY_LEFT:
    gimbal_command.gimbal_pan -= gimbal_increment;
    break;

  case 'd':
  case 'l':
  case KEY_RIGHT:
    gimbal_command.gimbal_pan += gimbal_increment;
    break;

  case 'm':
    gimbal_command.fpv_mode = !gimbal_command.fpv_mode;
    break;

  case 'o':
    gimbal_command.is_on = !gimbal_command.is_on;
    break;

  case 'r':
    gimbal_command.is_on       = true;
    gimbal_command.fpv_mode    = true;
    gimbal_command.gimbal_tilt = 1500;
    gimbal_command.gimbal_pan  = 1500;
    break;

    /* case 'r': */
    /*   reference.position.z = 1.0; */

    /*   if (turbo_remote_) { */
    /*     reference.position.z = 2.0; */
    /*   } */

    /*   remoteModeFly(reference); */
    /*   remote_hover_ = true; */
    /*   break; */

    /* case 'f': */
    /*   reference.position.z = -1.0; */

    /*   if (turbo_remote_) { */
    /*     reference.position.z = -2.0; */
    /*   } */

    /*   remoteModeFly(reference); */
    /*   remote_hover_ = true; */
    /*   break; */

    /* case 'q': */
    /*   reference.heading = 0.5; */

    /*   if (turbo_remote_) { */
    /*     reference.heading = 1.0; */
    /*   } */

    /*   remoteModeFly(reference); */
    /*   remote_hover_ = true; */
    /*   break; */

    /* case 'e': */
    /*   reference.heading = -0.5; */

    /*   if (turbo_remote_) { */
    /*     reference.heading = -1.0; */
    /*   } */

    /*   remoteModeFly(reference); */
    /*   remote_hover_ = true; */
    /*   break; */

    /* case 'T': */

    /*   bool is_flying_normally_; */

    /*   { */
    /*     std::scoped_lock lock(mutex_status_msg_); */
    /*     is_flying_normally_ = uav_status_.flying_normally; */
    /*   } */

    /*   if (is_flying_normally_) { */

    /*     if (turbo_remote_) { */

    /*       turbo_remote_                = false; */
    /*       string_service.request.value = old_constraints; */
    /*       service_set_constraints_.call(string_service, _service_num_calls_, _service_delay_); */
    /*       printServiceResult(string_service.response.success, string_service.response.message); */

    /*     } else { */

    /*       turbo_remote_ = true; */

    /*       { */
    /*         std::scoped_lock lock(mutex_status_msg_); */
    /*         old_constraints = uav_status_.constraints[0]; */
    /*       } */

    /*       string_service.request.value = _turbo_remote_constraints_; */
    /*       service_set_constraints_.call(string_service, _service_num_calls_, _service_delay_); */
    /*       printServiceResult(string_service.response.success, string_service.response.message); */
    /*     } */
    /*   } */

    /*   break; */

    /* case 'G': */


    /* { */
    /*   std::scoped_lock lock(mutex_status_msg_); */
    /*   is_flying_normally_ = uav_status_.flying_normally; */
    /* } */
    /*   if (is_flying_normally_) { */
    /*     remote_global_ = !remote_global_; */
    /*   } */

    /*   break; */


    /* default: */
    /*   if (remote_hover_) { */

    /*     service_hover_.call(trig, _service_num_calls_, _service_delay_); */
    /*     remote_hover_ = false; */
    /*   } */
    /*   break; */
  }

  if (gimbal_command.gimbal_pan > gimbal_max) {
    gimbal_command.gimbal_pan = gimbal_max;
  }
  if (gimbal_command.gimbal_tilt > gimbal_max) {
    gimbal_command.gimbal_tilt = gimbal_max;
  }

  if (gimbal_command.gimbal_pan < gimbal_min) {
    gimbal_command.gimbal_pan = gimbal_min;
  }
  if (gimbal_command.gimbal_tilt < gimbal_min) {
    gimbal_command.gimbal_tilt = gimbal_min;
  }

  ph_gimbal_state_.publish(gimbal_command);

  wattroff(win, A_BOLD);
}

//}

/* remoteModeFly() //{ */

void Status::remoteModeFly(const mrs_msgs::msg::Reference &ref_in) {

  auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

  if (remote_global_) {

    double      cmd_x;
    double      cmd_y;
    double      cmd_z;
    double      cmd_hdg;
    std::string odom_frame;

    {
      std::scoped_lock lock(mutex_status_msg_);
      cmd_x      = uav_status_.cmd_x;
      cmd_y      = uav_status_.cmd_y;
      cmd_z      = uav_status_.cmd_z;
      cmd_hdg    = uav_status_.cmd_hdg;
      odom_frame = uav_status_.odom_frame;
    }

    request->reference.position.x = cmd_x + ref_in.position.x;
    request->reference.position.y = cmd_y + ref_in.position.y;
    request->reference.position.z = cmd_z + ref_in.position.z;
    request->reference.heading    = cmd_hdg + ref_in.heading;
    request->header.frame_id      = odom_frame;

  } else {

    request->reference = ref_in;

    std::string uav_name;
    double      cmd_x;
    double      cmd_y;
    double      cmd_z;
    double      cmd_hdg;
    std::string odom_frame;

    {
      std::scoped_lock lock(mutex_status_msg_);
      uav_name   = uav_status_.uav_name;
      cmd_x      = uav_status_.cmd_x;
      cmd_y      = uav_status_.cmd_y;
      cmd_z      = uav_status_.cmd_z;
      cmd_hdg    = uav_status_.cmd_hdg;
      odom_frame = uav_status_.odom_frame;
    }

    mrs_msgs::msg::ReferenceStamped cmd_reference;

    cmd_reference.reference.position.x = cmd_x;
    cmd_reference.reference.position.y = cmd_y;
    cmd_reference.reference.position.z = cmd_z;
    cmd_reference.reference.heading    = cmd_hdg;
    cmd_reference.header.frame_id      = odom_frame;

    request->header.frame_id = uav_name + "/fcu_untilted";
    request->header.stamp    = clock_->now();

    auto response = transformer_->transformSingle(cmd_reference, request->header.frame_id);
    if (response) {
      cmd_reference = response.value();
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Transform failed when transforming cmd_reference.");
      return;
    }

    request->reference = cmd_reference.reference;
    request->reference.position.x += ref_in.position.x;
    request->reference.position.y += ref_in.position.y;
    request->reference.position.z += ref_in.position.z;
    request->reference.heading += ref_in.heading;
    request->header.frame_id = cmd_reference.header.frame_id;
  }

  request->header.stamp = clock_->now();

  auto response = service_goto_reference_.callSync(request);
}

//}

/* stringHandler() //{ */

void Status::stringHandler(WINDOW *win) {

  std::vector<std::string> string_vector;

  {
    std::scoped_lock lock(mutex_status_msg_);
    string_vector = uav_status_.custom_string_outputs;
  }


  // TODO - this section should probably not be here, but somewhere more sensible...
  uint8_t gnss_fix_type;
  uint8_t gnss_num_sats;
  double  gnss_pos_acc;
  double  gnss_status_rate;

  {
    std::scoped_lock lock(mutex_status_msg_);
    gnss_fix_type    = uav_status_.hw_api_gnss_fix_type;
    gnss_num_sats    = uav_status_.hw_api_gnss_num_sats;
    gnss_pos_acc     = uav_status_.hw_api_gnss_pos_acc;
    gnss_status_rate = uav_status_.hw_api_gnss_status_hz;
  }

  if (gnss_status_rate > 0.0) {
    std::string fix_string;

    if (gnss_fix_type < 1 || gnss_fix_type >= 8) {
      fix_string += "-r ";
    }
    fix_string += "Fix Type: ";

    switch (gnss_fix_type) {
    case 0: {
      fix_string += "NO GPS";
      break;
    }
    case 1: {
      fix_string += "NO FIX";
      break;
    }
    case 2: {
      fix_string += "2D FIX";
      break;
    }
    case 3: {
      fix_string += "3D FIX";
      break;
    }
    case 4: {
      fix_string += "3D SBAS FIX";
      break;
    }
    case 5: {
      fix_string += "RTK FLOAT";
      break;
    }
    case 6: {
      fix_string += "RTK FIX (INT)";
      break;
    }
    case 7: {
      fix_string += "STATIC - BASESTATION";
      break;
    }
    case 8: {
      fix_string += "PPP 3D FIX";
      break;
    }
    default: {
      fix_string += "UNKNOWN";
      break;
    }
    }

    std::string gnss_acc_string;
    if (gnss_pos_acc >= 100.0) {
      gnss_acc_string = "N/A";
    } else {
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << gnss_pos_acc;
      gnss_acc_string = stream.str();
    }
    std::string acc_string = "Num sats: " + to_string(gnss_num_sats) + " Acc: " + gnss_acc_string + " m";

    string_vector.push_back(fix_string);
    string_vector.push_back(acc_string);
  }
  // TODO - end of section

  if (string_vector.empty()) {
    werase(win);
    wnoutrefresh(win);
    return;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  for (unsigned long i = 0; i < string_vector.size(); i++) {

    int    tmp_color          = NORMAL;
    bool   blink              = false;
    string tmp_display_string = string_vector[i];

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

    if (mini_) {
      printCompressedLimitedString(win, (i) + 1, 1, tmp_display_string, 15);
    } else {
      printLimitedString(win, (i) + 1, 1, tmp_display_string, 30);
    }

    /* if (tmp_display_string.length() > 30) { */
    /*   printLimitedString(win, 1 + (3 * i) + 1, 1, tmp_display_string.substr(30), 30); */
    /* } */
    /* if (i < 2) { */
    /*   printLimitedString(win, (2 * i) + 2, 1, ("------------------------------"), 30); */
    /* } */

    wattroff(win, COLOR_PAIR(tmp_color));
    wattroff(win, A_BLINK);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* genericTopicHandler() //{ */

void Status::genericTopicHandler(WINDOW *win) {

  std::vector<mrs_msgs::msg::CustomTopic> custom_topic_vec;

  {
    std::scoped_lock lock(mutex_status_msg_);
    custom_topic_vec = uav_status_.custom_topics;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  if (!custom_topic_vec.empty()) {

    for (size_t i = 0; i < custom_topic_vec.size(); i++) {

      wattron(win, COLOR_PAIR(custom_topic_vec[i].topic_color));
      if (mini_) {
        printCompressedLimitedString(win, 1 + i, 1, custom_topic_vec[i].topic_name, 4);
        printLimitedDouble(win, 1 + i, 5, "%3.0f", custom_topic_vec[i].topic_hz, 1000);

      } else {
        printLimitedString(win, 1 + i, 1, custom_topic_vec[i].topic_name, 15);
        printLimitedDouble(win, 1 + i, 16, "%5.1f Hz", custom_topic_vec[i].topic_hz, 1000);
      }
      wattroff(win, COLOR_PAIR(custom_topic_vec[i].topic_color));
    }

  } else {

    werase(win);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* nodeStatsHandler() //{ */

void Status::nodeStatsHandler(WINDOW *win) {

  mrs_msgs::msg::NodeCpuLoad node_cpu_load_vec;

  double cpu_load_total;

  {
    std::scoped_lock lock(mutex_status_msg_);
    node_cpu_load_vec = uav_status_.node_cpu_loads;
    cpu_load_total    = uav_status_.cpu_load_total;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  if (!node_cpu_load_vec.node_names.empty()) {
    size_t tmp_num_lines = node_cpu_load_vec.node_names.size();
    if (tmp_num_lines > 9) {
      tmp_num_lines = 9;
    }

    wattron(win, COLOR_PAIR(GREEN));
    printLimitedString(win, 0, 1, "ROS Node CPU usage", 40);
    wattroff(win, COLOR_PAIR(GREEN));

    printLimitedDouble(win, 0, 37, "%5.1f", cpu_load_total, 9999);
    printLimitedString(win, 0, 43, "CPU %%", 6);
    for (size_t i = 0; i < tmp_num_lines; i++) {

      printLimitedString(win, 1 + i, 1, node_cpu_load_vec.node_names[i], 42);

      short tmp_color = GREEN;
      if (node_cpu_load_vec.cpu_loads[i] > 99.9) {
        tmp_color = RED;
      } else if (node_cpu_load_vec.cpu_loads[i] > 49.9) {
        tmp_color = YELLOW;
      }

      wattron(win, COLOR_PAIR(tmp_color));
      printLimitedDouble(win, 1 + i, 43, "%5.1f", node_cpu_load_vec.cpu_loads[i], 9999);
      wattroff(win, COLOR_PAIR(tmp_color));
    }


  } else {

    werase(win);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* uavStateHandler() //{ */

void Status::uavStateHandler(WINDOW *win) {

  double avg_rate;
  double color;
  double heading;
  double state_x;
  double state_y;
  double state_z;

  double cmd_x;
  double cmd_y;
  double cmd_z;
  double cmd_hdg;

  std::string odom_frame;
  std::string main_estimator;
  std::string horizontal_estimator;
  std::string vertical_estimator;
  std::string heading_estimator;
  std::string agl_estimator;
  double      max_flight_z;

  bool null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    avg_rate   = uav_status_.odom_hz;
    color      = uav_status_.odom_color;
    heading    = uav_status_.odom_hdg;
    state_x    = uav_status_.odom_x;
    state_y    = uav_status_.odom_y;
    state_z    = uav_status_.odom_z;
    odom_frame = uav_status_.odom_frame;

    cmd_x   = uav_status_.cmd_x;
    cmd_y   = uav_status_.cmd_y;
    cmd_z   = uav_status_.cmd_z;
    cmd_hdg = uav_status_.cmd_hdg;

    uav_status_.odom_estimators.empty() ? main_estimator = "NONE" : main_estimator = uav_status_.odom_estimators[0];

    horizontal_estimator = uav_status_.horizontal_estimator;
    vertical_estimator   = uav_status_.vertical_estimator;
    heading_estimator    = uav_status_.heading_estimator;
    agl_estimator        = uav_status_.agl_estimator;

    max_flight_z = uav_status_.max_flight_z;

    null_tracker = uav_status_.null_tracker;
  }

  double cerr_x   = std::fabs(state_x - cmd_x);
  double cerr_y   = std::fabs(state_y - cmd_y);
  double cerr_z   = std::fabs(state_z - cmd_z);
  double cerr_hdg = fabs(radians::diff(heading, cmd_hdg));

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  /* mini //{ */

  if (mini_) {
    printLimitedDouble(win, 0, 1, "Odm %3.0f", avg_rate, 1000);

    if (avg_rate == 0) {

      printNoData(win, 0, 1);

    } else {

      printLimitedDouble(win, 1, 1, "%4.0f", state_x, 1000);
      printLimitedDouble(win, 2, 1, "%4.0f", state_y, 1000);
      printLimitedDouble(win, 3, 1, "%4.0f", state_z, 1000);
      printLimitedDouble(win, 4, 1, "%4.1f", heading, 1000);

      printLimitedString(win, 1, 6, main_estimator, 2);
    }
  }

  //}

  /* standard //{ */

  else {

    printLimitedDouble(win, 0, 12, "Odom %5.1f Hz", avg_rate, 1000);

    if (avg_rate == 0) {

      printNoData(win, 0, 1);

    } else {

      printLimitedDouble(win, 1, 1, "X %7.2f", state_x, 1000);
      printLimitedDouble(win, 2, 1, "Y %7.2f", state_y, 1000);
      printLimitedDouble(win, 3, 1, "Z %7.2f", state_z, 1000);
      printLimitedDouble(win, 4, 1, "hdg %5.2f", heading, 1000);

      if (!null_tracker) {
        wattron(win, COLOR_PAIR(NORMAL));
        mvwprintw(win, 5, 1, "C/E");

        if (cerr_x < 0.5) {
          wattron(win, COLOR_PAIR(GREEN));
        } else if (cerr_x < 1.0) {
          wattron(win, COLOR_PAIR(YELLOW));
        } else {
          wattron(win, COLOR_PAIR(RED));
        }
        printLimitedDouble(win, 5, 5, "X%1.1f", cerr_x, 10);


        if (cerr_y < 0.5) {
          wattron(win, COLOR_PAIR(GREEN));
        } else if (cerr_y < 1.0) {
          wattron(win, COLOR_PAIR(YELLOW));
        } else {
          wattron(win, COLOR_PAIR(RED));
        }
        printLimitedDouble(win, 5, 10, "Y%1.1f", cerr_y, 10);

        if (cerr_z < 0.5) {
          wattron(win, COLOR_PAIR(GREEN));
        } else if (cerr_z < 1.0) {
          wattron(win, COLOR_PAIR(YELLOW));
        } else {
          wattron(win, COLOR_PAIR(RED));
        }
        printLimitedDouble(win, 5, 15, "Z%1.1f", cerr_z, 10);

        if (cerr_hdg < 0.2) {
          wattron(win, COLOR_PAIR(GREEN));
        } else if (cerr_hdg < 0.4) {
          wattron(win, COLOR_PAIR(YELLOW));
        } else {
          wattron(win, COLOR_PAIR(RED));
        }
        printLimitedDouble(win, 5, 20, "H%1.1f", cerr_hdg, 10);

        wattron(win, COLOR_PAIR(color));
      }

      /* horizontal_estimator = uav_status_.horizontal_estimator; */
      /* vertical_estimator = uav_status_.vertical_estimator; */
      /* heading_estimator = uav_status_.heading_estimator; */
      /* agl_estimator = uav_status_.agl_estimator; */
      printLimitedString(win, 1, 11, main_estimator, 14);

      switch (estimator_display_counter_) {
      case 0: {
        printLimitedString(win, 2, 11, "hor: " + horizontal_estimator, 14);
        break;
      }
      case 1: {
        printLimitedString(win, 2, 11, "ver: " + vertical_estimator, 14);
        break;
      }
      case 2: {
        printLimitedString(win, 2, 11, "hdg: " + heading_estimator, 14);
        break;
      }
      }


      printLimitedString(win, 4, 11, "ag: " + agl_estimator, 14);

      double dist_to_max_z = max_flight_z - state_z;
      if (dist_to_max_z < 0.0) {
        wattron(win, COLOR_PAIR(RED));
        wattron(win, A_BLINK);
      } else if (dist_to_max_z < 0.3) {
        wattron(win, COLOR_PAIR(RED));
      } else if (dist_to_max_z < 1.0) {
        wattron(win, COLOR_PAIR(YELLOW));
      } else {
        wattron(win, COLOR_PAIR(GREEN));
      }

      printLimitedDouble(win, 3, 11, "Max: %5.1f", max_flight_z, 1000);
      wattron(win, COLOR_PAIR(color));
      wattroff(win, A_BLINK);
    }
  }

  //}

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

//}

/* controlManagerHandler() //{ */

void Status::controlManagerHandler(WINDOW *win) {

  int16_t color;
  bool    null_tracker;
  double  rate;
  string  curr_controller;
  string  curr_tracker;
  string  curr_gains;
  string  curr_constraints;
  bool    callbacks_enabled;
  bool    rc_mode;
  bool    have_goal;
  bool    tracking_trajectory;

  {
    std::scoped_lock lock(mutex_status_msg_);
    rate  = uav_status_.control_manager_diag_hz;
    color = uav_status_.control_manager_diag_color;

    uav_status_.controllers.empty() ? curr_controller = "NONE" : curr_controller = uav_status_.controllers[0];
    uav_status_.trackers.empty() ? curr_tracker = "NONE" : curr_tracker = uav_status_.trackers[0];
    uav_status_.gains.empty() ? curr_gains = "NONE" : curr_gains = uav_status_.gains[0];
    uav_status_.constraints.empty() ? curr_constraints = "NONE" : curr_constraints = uav_status_.constraints[0];

    callbacks_enabled   = uav_status_.callbacks_enabled;
    rc_mode             = uav_status_.rc_mode;
    have_goal           = uav_status_.have_goal;
    tracking_trajectory = uav_status_.tracking_trajectory;
    null_tracker        = uav_status_.null_tracker;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, COLOR_PAIR(color));

  /* mini //{ */

  if (mini_) {
    printLimitedDouble(win, 0, 1, "Ctr %3.0f", rate, 1000);

    if (rate == 0.0) {

      printNoData(win, 0, 1);
      wattron(win, COLOR_PAIR(RED));
      mvwprintw(win, 1, 1, "ERR");
      mvwprintw(win, 2, 1, "ERR");
      wattroff(win, COLOR_PAIR(color));

    } else {

      if (curr_controller != "Se3Controller" && curr_controller != "MpcController") {
        wattron(win, COLOR_PAIR(RED));
        printLimitedString(win, 1, 1, curr_controller, 3);
      } else {
        printLimitedString(win, 1, 1, curr_controller, 3);
        wattron(win, COLOR_PAIR(NORMAL));
        mvwprintw(win, 1, 4, "%s", "/");
      }

      wattron(win, COLOR_PAIR(color));

      if (null_tracker) {
        curr_tracker = "NlT";
      }

      if (curr_tracker != "MpcTracker") {
        if (curr_tracker == "LandoffTracker" && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        } else {
          wattron(win, COLOR_PAIR(RED));
        }

        printLimitedString(win, 2, 1, curr_tracker, 3);

      } else {
        printLimitedString(win, 2, 1, curr_tracker, 3);
        wattron(win, COLOR_PAIR(NORMAL));
        mvwprintw(win, 2, 4, "%s", "/");
        wattron(win, COLOR_PAIR(color));
      }

      printLimitedString(win, 1, 5, curr_gains, 3);
      printLimitedString(win, 2, 5, curr_constraints, 3);
    }
  }

  //}

  /* standrad //{ */

  else {

    /* printLimitedString(win, 0, 10, "Control Manager", 15); */
    printLimitedDouble(win, 0, 1, "Control Manager %5.1f Hz", rate, 1000);

    if (rate == 0.0) {

      printNoData(win, 0, 1);

      wattron(win, COLOR_PAIR(RED));
      mvwprintw(win, 1, 1, "NO_CONTROLLER");
      mvwprintw(win, 2, 1, "NO_TRACKER");
      wattroff(win, COLOR_PAIR(color));

    } else {
      if (curr_controller != "Se3Controller" && curr_controller != "MpcController") {
        wattron(win, COLOR_PAIR(RED));
      }
      printLimitedString(win, 1, 1, curr_controller, 13);
      wattron(win, COLOR_PAIR(NORMAL));
      printLimitedString(win, 1, 1 + std::min(int(curr_controller.length()), 13), "/" + curr_gains, 10);
      wattron(win, COLOR_PAIR(color));

      if (null_tracker) {
        curr_tracker = "NullTracker";
      }

      if (curr_tracker != "MpcTracker") {
        if (curr_tracker == "LandoffTracker" && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        } else {
          wattron(win, COLOR_PAIR(RED));
        }
      }

      printLimitedString(win, 2, 1, curr_tracker, 13);
      wattron(win, COLOR_PAIR(NORMAL));
      printLimitedString(win, 2, 1 + std::min(int(curr_tracker.length()), 13), "/" + curr_constraints, 8);
      wattron(win, COLOR_PAIR(color));
    }

    if (rc_mode) {
      wattron(win, A_BLINK);
      wattron(win, COLOR_PAIR(RED));
      mvwprintw(win, 1, 18, "RC_MODE");
      wattroff(win, COLOR_PAIR(RED));
      wattroff(win, A_BLINK);

    } else if (!callbacks_enabled) {
      wattron(win, COLOR_PAIR(RED));
      mvwprintw(win, 1, 20, "NO_CB");
      wattroff(win, COLOR_PAIR(RED));
    }

    if (tracking_trajectory) {
      wattron(win, COLOR_PAIR(GREEN));
      mvwprintw(win, 2, 21, "TRAJ");
      wattroff(win, COLOR_PAIR(GREEN));

    } else if (have_goal) {
      wattron(win, COLOR_PAIR(GREEN));
      mvwprintw(win, 2, 21, "GOTO");
      wattroff(win, COLOR_PAIR(GREEN));

    } else {
      wattron(win, COLOR_PAIR(YELLOW));
      mvwprintw(win, 2, 21, "IDLE");
      wattroff(win, COLOR_PAIR(YELLOW));
    }
  }

  //}

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* hwApiStateHander() //{ */

void Status::hwApiStateHander(WINDOW *win) {

  int16_t     color;
  double      hw_api_rate;
  double      state_rate;
  double      cmd_rate;
  double      battery_rate;
  bool        gnss_ok;
  bool        armed;
  std::string mode;
  double      battery_volt;
  double      battery_curr;
  double      battery_wh_drained;
  double      thrust;
  double      mass_estimate;
  double      mass_set;
  double      gnss_qual;
  double      mag_norm;
  double      mag_norm_rate;

  {
    std::scoped_lock lock(mutex_status_msg_);
    color              = uav_status_.hw_api_color;
    hw_api_rate        = uav_status_.hw_api_hz;
    state_rate         = uav_status_.hw_api_state_hz;
    cmd_rate           = uav_status_.hw_api_cmd_hz;
    battery_rate       = uav_status_.hw_api_battery_hz;
    gnss_ok            = uav_status_.hw_api_gnss_ok;
    armed              = uav_status_.hw_api_armed;
    mode               = uav_status_.hw_api_mode;
    battery_volt       = uav_status_.battery_volt;
    battery_curr       = uav_status_.battery_curr;
    battery_wh_drained = uav_status_.battery_wh_drained;
    thrust             = uav_status_.thrust;
    mass_estimate      = uav_status_.mass_estimate;
    mass_set           = uav_status_.mass_set;
    gnss_qual          = uav_status_.hw_api_gnss_qual;
    mag_norm           = uav_status_.mag_norm;
    mag_norm_rate      = uav_status_.mag_norm_hz;
  }

  std::string tmp_string;

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  /* mini //{ */

  if (mini_) {
    printLimitedDouble(win, 0, 1, "Mav %3.0f", hw_api_rate, 1000);
    wattroff(win, COLOR_PAIR(color));

    if (hw_api_rate == 0) {
      printNoData(win, 0, 1);
    }

    if (state_rate == 0) {

      wattron(win, COLOR_PAIR(RED));
      printLimitedString(win, 1, 1, "ERR", 3);
      printLimitedString(win, 2, 1, "ERR", 3);
      wattroff(win, COLOR_PAIR(RED));

    } else {

      if (armed) {
        tmp_string = "ARM";
        wattron(win, COLOR_PAIR(GREEN));
      } else {
        tmp_string = "DIS";
        wattron(win, COLOR_PAIR(RED));
      }

      printLimitedString(win, 1, 1, tmp_string, 15);
      wattron(win, COLOR_PAIR(color));

      if (mode != "OFFBOARD") {
        wattron(win, COLOR_PAIR(RED));
      }

      printLimitedString(win, 2, 1, mode, 3);
      wattron(win, COLOR_PAIR(color));
    }

    if (battery_rate == 0) {

      printLimitedString(win, 3, 1, "ERR", 3);

    } else {

      wattron(win, COLOR_PAIR(GREEN));

      (battery_volt > 17.0) ? (battery_volt = battery_volt / 6) : (battery_volt = battery_volt / 4);

      if (battery_volt < 3.6) {
        wattron(win, COLOR_PAIR(RED));
      } else if (battery_volt < 3.7 && color != RED) {
        wattron(win, COLOR_PAIR(YELLOW));
      }
      printLimitedString(win, 3, 1, "Bat", 3);
    }


    if (cmd_rate == 0) {

      printLimitedString(win, 3, 5, "ERR", 3);

    } else {

      if (thrust > 0.75) {
        wattron(win, COLOR_PAIR(RED));
      } else if (thrust > 0.65 && color != RED) {
        wattron(win, COLOR_PAIR(YELLOW));
      }
      printLimitedDouble(win, 3, 5, ".%2.0f", thrust * 100, 100);
      wattron(win, COLOR_PAIR(color));

      color            = GREEN;
      double mass_diff = fabs(mass_estimate - mass_set) / mass_set;

      if (mass_diff > 0.3) {

        color = RED;

      } else if (mass_diff > 0.2) {

        color = YELLOW;
      }

      printLimitedDouble(win, 4, 1, "%4.1f kg", mass_estimate, 99.99);
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(RED));
      printLimitedString(win, 1, 5, "GPS", 6);
      wattroff(win, COLOR_PAIR(RED));

    } else {

      wattron(win, COLOR_PAIR(GREEN));
      printLimitedString(win, 1, 5, "GPS", 6);
      wattroff(win, COLOR_PAIR(GREEN));

      color = RED;

      if (gnss_qual < 5.0) {
        color = GREEN;
      } else if (gnss_qual < 10.0) {
        color = YELLOW;
      }

      wattron(win, COLOR_PAIR(color));

      if (gnss_qual < 10.0) {
        printLimitedDouble(win, 2, 5, "%3.1f", gnss_qual, 9.9);
      } else {
        printLimitedString(win, 2, 5, ">10", 3);
      }
      wattroff(win, COLOR_PAIR(color));
    }

  }

  //}

  /* standard  //{ */

  else {

    printLimitedDouble(win, 0, 9, "HW Api %5.1f Hz", hw_api_rate, 1000);
    wattroff(win, COLOR_PAIR(color));

    if (hw_api_rate == 0) {

      printNoData(win, 0, 1);
    }

    if (state_rate == 0) {

      wattron(win, COLOR_PAIR(RED));
      printLimitedString(win, 1, 1, "State: ", 15);
      printNoData(win, 1, 9);
      printLimitedString(win, 2, 1, "Mode: ", 15);
      printNoData(win, 1, 9);
      wattroff(win, COLOR_PAIR(RED));

    } else {

      if (armed) {
        tmp_string = "ARMED";
        wattron(win, COLOR_PAIR(GREEN));
      } else {
        tmp_string = "DISARMED";
        wattron(win, COLOR_PAIR(RED));
      }

      printLimitedString(win, 1, 1, "State: " + tmp_string, 15);
      wattron(win, COLOR_PAIR(GREEN));

      if (mode != "OFFBOARD") {
        wattron(win, COLOR_PAIR(RED));
      }

      printLimitedString(win, 2, 1, "Mode:  " + mode, 15);
      wattron(win, COLOR_PAIR(color));
    }

    if (battery_rate == 0) {

      printNoData(win, 4, 1, "Batt:  ");

    } else {

      wattron(win, COLOR_PAIR(GREEN));

      (battery_volt > 17.0) ? (battery_volt = battery_volt / 6) : (battery_volt = battery_volt / 4);

      if (battery_volt < 3.6) {
        wattron(win, COLOR_PAIR(RED));
      } else if (battery_volt < 3.7 && color != RED) {
        wattron(win, COLOR_PAIR(YELLOW));
      }
      printLimitedDouble(win, 4, 1, "%4.2fV ", battery_volt, 10);
      printLimitedDouble(win, 4, 8, "%5.2fA", battery_curr, 100);
      printLimitedDouble(win, 4, 15, " %4.1f Wh", battery_wh_drained, 100);
    }

    if (mag_norm_rate == 0) {

      printNoData(win, 3, 1, "Mag:  ");

    } else {

      wattron(win, COLOR_PAIR(GREEN));

      if (mag_norm > 0.9 || mag_norm < 0.25) {
        wattron(win, COLOR_PAIR(RED));
      } else if (mag_norm > 0.65) {
        wattron(win, COLOR_PAIR(YELLOW));
      }
      printLimitedDouble(win, 3, 1, "Mag: %4.2f", mag_norm, 9.99);
    }

    if (cmd_rate == 0) {

      printNoData(win, 5, 1, "Thrst: ");

    } else {

      wattron(win, COLOR_PAIR(GREEN));

      if (thrust > 0.75) {
        wattron(win, COLOR_PAIR(RED));
      } else if (thrust > 0.65 && color != RED) {
        wattron(win, COLOR_PAIR(YELLOW));
      }
      printLimitedDouble(win, 5, 1, "Thrst: %4.2f", thrust, 1.01);
      wattron(win, COLOR_PAIR(color));

      color            = GREEN;
      double mass_diff = fabs(mass_estimate - mass_set) / mass_set;

      if (mass_diff > 0.3) {

        color = RED;

      } else if (mass_diff > 0.2) {

        color = YELLOW;
      }

      if (mass_set > 10.0 || mass_estimate > 10.0) {

        wattron(win, COLOR_PAIR(NORMAL));
        printLimitedDouble(win, 5, 13, "%.1f/", mass_set, 99.99);
        wattron(win, COLOR_PAIR(color));
        printLimitedDouble(win, 5, 18, "%.1f", mass_estimate, 99.99);
        printLimitedString(win, 5, 22, "kg", 2);

      } else {

        wattron(win, COLOR_PAIR(NORMAL));
        printLimitedDouble(win, 5, 15, "%.1f/", mass_set, 99.99);
        wattron(win, COLOR_PAIR(color));
        printLimitedDouble(win, 5, 19, "%.1f", mass_estimate, 99.99);
        printLimitedString(win, 5, 22, "kg", 2);
      }
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(RED));
      printLimitedString(win, 1, 18, "NO_GPS", 6);
      wattroff(win, COLOR_PAIR(RED));

    } else {

      wattron(win, COLOR_PAIR(GREEN));
      printLimitedString(win, 1, 18, "GPS_OK", 6);
      wattroff(win, COLOR_PAIR(GREEN));

      color = RED;

      if (gnss_qual < 5.0) {
        color = GREEN;
      } else if (gnss_qual < 10.0) {
        color = YELLOW;
      }

      wattron(win, COLOR_PAIR(color));
      printLimitedDouble(win, 2, 17, "Q: %4.1f", gnss_qual, 99.9);
      wattroff(win, COLOR_PAIR(color));
    }
  }

  //}

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}
//}

/* topLineHandler() //{ */

void Status::topLineHandler(WINDOW *win) {

  werase(win);
  int secs_flown;

  {
    std::scoped_lock lock(mutex_status_msg_);
    secs_flown = uav_status_.secs_flown;
  }

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  printLimitedInt(win, 0, 0, "ToF: %i", secs_flown, 1000);

  std::string uav_name;
  std::string uav_type;

  bool collision_avoidance_enabled;

  uint16_t num_other_uavs;

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_name                     = uav_status_.uav_name;
    uav_type                     = uav_status_.uav_type;
    collision_avoidance_enabled  = uav_status_.collision_avoidance_enabled;
    avoiding_collision_          = uav_status_.avoiding_collision;
    automatic_start_can_takeoff_ = uav_status_.automatic_start_can_takeoff;
    null_tracker_                = uav_status_.null_tracker;
    num_other_uavs               = uav_status_.num_other_uavs;
  }

  double tmp_time       = (clock_->now() - last_time_got_data_).seconds();
  double tmp_short_time = (clock_->now() - last_time_got_short_data_).seconds();

  if (tmp_short_time < 3.0) {
    have_short_data_ = true;
  } else {
    have_short_data_ = false;
  }

  if (tmp_short_time >= 99.9) {
    tmp_short_time = 99.9;
  }

  if (tmp_time > 3.0 && have_data_) {
    have_data_ = false;
    setupColors(have_data_);
  }

  if (tmp_time < 3.0 && !have_data_) {
    have_data_ = true;
    setupColors(have_data_);
  }

  if (tmp_time >= 99.9) {
    tmp_time = 99.9;
  }

  mvwprintw(win, 0, 10, " %s %s ", uav_name.c_str(), uav_type.c_str());

  if (!mini_) {
    /* printLimitedDouble(win, 0, 94, "%3.1f", tmp_time, 100); */
    /* printLimitedDouble(win, 0, 90, "%3.1f", tmp_short_time, 100); */

    if (collision_avoidance_enabled) {
      if (avoiding_collision_) {
        wattron(win, COLOR_PAIR(RED));
        wattron(win, A_BLINK);
        mvwprintw(win, 0, 26, "!! AVOIDING COLLISION !!");
        wattroff(win, COLOR_PAIR(RED));
        wattroff(win, A_BLINK);
      } else {
        wattron(win, COLOR_PAIR(GREEN));
        mvwprintw(win, 0, 26, "COL AVOID ENABLED,");
        if (num_other_uavs == 0) {
          wattron(win, COLOR_PAIR(RED));
        }
        mvwprintw(win, 0, 45, "UAVs: ");
        printLimitedInt(win, 0, 51, "%i", num_other_uavs, 100);
        wattroff(win, COLOR_PAIR(GREEN));
        wattroff(win, COLOR_PAIR(RED));
      }
    } else {
      wattron(win, COLOR_PAIR(RED));
      mvwprintw(win, 0, 26, "COL AVOID DISABLED");
      wattroff(win, COLOR_PAIR(RED));
    }
  } else {

    if (collision_avoidance_enabled) {

      if (avoiding_collision_) {
        wattron(win, COLOR_PAIR(RED));
        wattron(win, A_BLINK);
        mvwprintw(win, 0, 22, "!AVOIDING!");
        wattroff(win, COLOR_PAIR(RED));
        wattroff(win, A_BLINK);
      } else {
        wattron(win, COLOR_PAIR(GREEN));
        mvwprintw(win, 0, 27, "C/A");
        if (num_other_uavs == 0) {
          wattron(win, COLOR_PAIR(RED));
        }
        printLimitedInt(win, 0, 31, "%i", num_other_uavs, 100);
        wattroff(win, COLOR_PAIR(GREEN));
        wattroff(win, COLOR_PAIR(RED));
      }
    } else {
      wattron(win, COLOR_PAIR(RED));
      mvwprintw(win, 0, 27, "C/A");
      wattroff(win, COLOR_PAIR(RED));
    }
  }

  if (!have_data_) {
    wattron(win, A_BLINK);
    wattron(win, COLOR_PAIR(ALWAYS_RED));
    mvwprintw(win, 0, 0, "!NO MSGS!");
    wattroff(win, COLOR_PAIR(ALWAYS_RED));
    wattroff(win, A_BLINK);
  }

  int mins = secs_flown / 60;
  int secs = secs_flown % 60;

  mvwprintw(win, 0, 0, "ToF: %i:%02i", mins, secs);
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

//}

/* generalInfoHandeler() //{ */

void Status::generalInfoHandeler(WINDOW *win) {

  werase(win);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(NORMAL));
  wattroff(win, COLOR_PAIR(NORMAL));
  wattroff(win, A_STANDOUT);
  printBox(win);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  printCpuLoad(win);
  /* printCpuTemp(win); */
  printMemLoad(win);
  if (!mini_) {
    printCpuFreq(win);
  }
  printDiskSpace(win);

  wnoutrefresh(win);
}

//}

//}

/* callbackUavStatus() //{ */

void Status::callbackUavStatus(const mrs_msgs::msg::UavStatus::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_ = *msg;
  }

  last_time_got_data_       = clock_->now();
  last_time_got_short_data_ = clock_->now();
}

//}

/* callbackUavStatusShort() //{ */

void Status::callbackUavStatusShort(const mrs_msgs::msg::UavStatusShort::ConstSharedPtr msg) {

  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.odom_x     = msg->odom_x;
    uav_status_.odom_y     = msg->odom_y;
    uav_status_.odom_z     = msg->odom_z;
    uav_status_.odom_hdg   = msg->odom_hdg;
    uav_status_.odom_color = msg->odom_color;
    uav_status_.odom_hz    = msg->odom_hz;

    uav_status_.cmd_x   = msg->cmd_x;
    uav_status_.cmd_y   = msg->cmd_y;
    uav_status_.cmd_z   = msg->cmd_z;
    uav_status_.cmd_hdg = msg->cmd_hdg;
  }

  last_time_got_short_data_ = clock_->now();
}

//}

/* prefillUavStatus() //{ */

void Status::prefillUavStatus() {

  std::scoped_lock lock(mutex_status_msg_);

  uav_status_.uav_name                = "N/A";
  uav_status_.uav_type                = "N/A";
  uav_status_.uav_mass                = 0.0;
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
  uav_status_.max_flight_z          = 0.0;
  uav_status_.cpu_load              = 0.0;
  uav_status_.cpu_ghz               = 0.0;
  uav_status_.free_ram              = 0.0;
  uav_status_.free_hdd              = 0.0;
  uav_status_.hw_api_hz             = 0.0;
  uav_status_.hw_api_armed          = false;
  uav_status_.hw_api_mode           = "N/A";
  uav_status_.hw_api_gnss_ok        = false;
  uav_status_.hw_api_gnss_qual      = 0.0;
  uav_status_.hw_api_gnss_fix_type  = 0.0;
  uav_status_.hw_api_gnss_num_sats  = 0.0;
  uav_status_.hw_api_gnss_pos_acc   = 0.0;
  uav_status_.hw_api_gnss_status_hz = 0.0;
  uav_status_.battery_volt          = 0.0;
  uav_status_.battery_curr          = 0.0;
  uav_status_.thrust                = 0.0;
  uav_status_.mass_estimate         = 0.0;
  uav_status_.mass_set              = 0.0;
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

/* MENU SETUP //{ */

/* setupMainMenu() //{ */

void Status::setupMainMenu() {

  service_vec_.clear();

  bool null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    null_tracker = uav_status_.null_tracker;
  }

  for (unsigned long i = 0; i < service_input_vec_.size(); i++) {

    if (null_tracker && (i == 0 || i == 1)) {
      continue; // disable land and land home if we are not flying
    }

    if (!null_tracker && i == 2) {

      continue; // disable takeoff if flying
    }

    std::vector<std::string> results;
    boost::split(results, service_input_vec_[i], [](char c) { return c == ' '; }); // split the input string into words and put them in results vector

    for (unsigned long j = 2; j < results.size(); j++) {
      results[1] = results[1] + " " + results[j];
    }

    string service_name;

    if (results[0].at(0) == '/') {
      service_name = results[0];

    } else {


      std::string uav_name;

      {
        std::scoped_lock lock(mutex_status_msg_);
        uav_name = uav_status_.uav_name;
      }

      service_name = "/" + uav_name + "/" + results[0];
    }

    service tmp_service(service_name, results[1]);

    tmp_service.service_client = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, service_name);

    service_vec_.push_back(tmp_service);
  }

  main_menu_text_.clear();

  for (unsigned long i = 0; i < service_vec_.size(); i++) {
    main_menu_text_.push_back(service_vec_[i].service_display_name);
  }

  main_menu_text_.push_back("Set Constraints");
  main_menu_text_.push_back("Set Gains");
  main_menu_text_.push_back("Set Controller");
  main_menu_text_.push_back("Set Tracker");
  main_menu_text_.push_back("Set Estimator");

  Menu menu(1, 32, main_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* setupGotoMenu() //{ */

void Status::setupGotoMenu() {

  std::string odom_frame;

  {
    std::scoped_lock lock(mutex_status_msg_);
    odom_frame = uav_status_.odom_frame;
  }

  goto_menu_inputs_.clear();
  goto_menu_text_.clear();
  goto_menu_text_.push_back(" X:                ");
  goto_menu_text_.push_back(" Y:                ");
  goto_menu_text_.push_back(" Z:                ");
  goto_menu_text_.push_back(" hdg:              ");
  goto_menu_text_.push_back(" " + odom_frame + " ");

  Menu menu(1, 32, goto_menu_text_);
  menu_vec_.push_back(menu);

  for (int i = 0; i < 4; i++) {
    InputBox tmpbox(8, menu.getWin(), goto_double_vec_[i]);
    goto_menu_inputs_.push_back(tmpbox);
  }
}

//}

/* setupDisplayMenu() //{ */

void Status::setupDisplayMenu() {

  setupDisplayText();

  Menu menu(1, 32, display_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* setupDisplayText() //{ */

void Status::setupDisplayText() {

  display_menu_text_.clear();

  char                     command[50] = "tmux list-windows | cut -d' ' -f-2";
  std::string              response    = callTerminal(command);
  std::vector<std::string> results;
  boost::split(results, response, boost::is_any_of("\n"));

  for (size_t i = 0; i < results.size() - 1; i++) {
    display_menu_text_.push_back("[ ] " + results[i]);
  }

  for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
    display_menu_text_[selected_tmux_window_[i]][1] = '*';
  }
}

//}

//}

/* PRINT FUNCTIONS //{ */

/* printMemLoad() //{ */

void Status::printMemLoad(WINDOW *win) {

  double total_ram;
  double free_ram;

  {
    std::scoped_lock lock(mutex_status_msg_);
    free_ram  = uav_status_.free_ram;
    total_ram = uav_status_.total_ram;
  }

  double used_ram = total_ram - free_ram;

  int    tmp_color = GREEN;
  double ram_ratio = used_ram / total_ram;
  if (ram_ratio > 0.7) {
    tmp_color = RED;
    wattron(win, A_BLINK);
  } else if (ram_ratio > 0.5) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini_) {
    printLimitedString(win, 2, 1, "RAM", 3);
  } else {
    printLimitedDouble(win, 2, 1, "RAM: %4.1f G", free_ram, 100);
  }
  wattroff(win, A_BLINK);
}

//}

/* printCpuLoad() //{ */

void Status::printCpuLoad(WINDOW *win) {

  double cpu_load;
  {
    std::scoped_lock lock(mutex_status_msg_);
    cpu_load = uav_status_.cpu_load;
  }

  int tmp_color = GREEN;
  if (cpu_load > 80.0) {
    tmp_color = RED;
  } else if (cpu_load > 60.0) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini_) {
    printLimitedString(win, 1, 1, "CPU", 3);
  } else {
    printLimitedDouble(win, 1, 1, "CPU: %4.1f %%", cpu_load, 99.9);
  }
}
//}

/* printCpuTemp() //{ */

void Status::printCpuTemp(WINDOW *win) {

  double cpu_temp;
  {
    std::scoped_lock lock(mutex_status_msg_);
    cpu_temp = uav_status_.cpu_temperature;
  }

  int tmp_color = GREEN;
  if (cpu_temp > 90.0) {
    tmp_color = RED;
  } else if (cpu_temp > 75.0) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini_) {
    printLimitedDouble(win, 0, 1, "%3.0f °C", cpu_temp, 999.9);
  } else {
    printLimitedDouble(win, 0, 1, "%5.1f °C", cpu_temp, 999.9);
  }
}

//}

/* printCpuFreq() //{ */

void Status::printCpuFreq(WINDOW *win) {

  double avg_cpu_ghz;
  {
    std::scoped_lock lock(mutex_status_msg_);
    avg_cpu_ghz = uav_status_.cpu_ghz;
  }

  wattron(win, COLOR_PAIR(GREEN));
  printLimitedDouble(win, 1, 16, "%4.2f GHz", avg_cpu_ghz, 10);
}

//}

/* printDiskSpace() //{ */

void Status::printDiskSpace(WINDOW *win) {

  int gigas;
  {
    std::scoped_lock lock(mutex_status_msg_);
    gigas = uav_status_.free_hdd;
  }

  wattron(win, COLOR_PAIR(GREEN));
  if (gigas < 200 || gigas != last_gigas_) {
    wattron(win, COLOR_PAIR(YELLOW));
  }
  if (gigas < 100) {
    wattron(win, COLOR_PAIR(RED));
    if (mini_) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedDouble(win, 2, 5, "%3.1f", double(gigas) / 10, 10);
    } else {
      printLimitedDouble(win, 2, 14, "HDD: %3.1f G", double(gigas) / 10, 10);
    }
  }
  if (gigas < 1000) {
    if (mini_) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 6, "%i", gigas / 10, 1000);
    } else {
      printLimitedInt(win, 2, 14, "HDD:  %i G", gigas / 10, 1000);
    }
  }
  if (gigas < 10000) {

    if (mini_) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 5, "%i", gigas / 10, 1000);
    } else {
      printLimitedInt(win, 2, 14, "HDD: %i G", gigas / 10, 1000);
    }
  } else {
    if (mini_) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 5, "%i T", gigas / 10000, 10);
    } else {
      printLimitedDouble(win, 2, 14, "HDD: %3.1f T", gigas / 10000.0, 1000);
    }
  }
}

//}

/* printServiceResult() //{ */

void Status::printServiceResult(bool success, string msg) {

  if (_light_) {
    wattron(bottom_window_, A_STANDOUT);
  }

  werase(bottom_window_);

  wattron(bottom_window_, A_BOLD);
  wattron(bottom_window_, COLOR_PAIR(GREEN));


  if (success) {

    printLimitedString(bottom_window_, 0, 0, "Service call success: " + msg, 120);

  } else {

    wattron(bottom_window_, COLOR_PAIR(RED));

    printLimitedString(bottom_window_, 0, 0, "Service call failed: " + msg, 120);

    wattroff(bottom_window_, COLOR_PAIR(RED));
  }

  bottom_window_clear_time_ = clock_->now();

  wattroff(bottom_window_, COLOR_PAIR(GREEN));
  wattroff(bottom_window_, A_BOLD);
}

//}

/* printLimitedInt() //{ */

void Status::printLimitedInt(WINDOW *win, int y, int x, string str_in, int num, int limit) {

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

  const char *format = str_in.c_str();

  mvwprintw(win, y, x, format, num);
}

//}

/* printLimitedDouble() //{ */

void Status::printLimitedDouble(WINDOW *win, int y, int x, string str_in, double num, double limit) {

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

  const char *format = str_in.c_str();

  mvwprintw(win, y, x, format, num);
}

//}

/* printLimitedString() //{ */

void Status::printLimitedString(WINDOW *win, int y, int x, string str_in, unsigned long limit) {

  if (str_in.length() > limit) {
    str_in.resize(limit);
  }

  const char *format = str_in.c_str();

  mvwprintw(win, y, x, format);
}

//}

/* printCompressedLimitedString() //{ */

void Status::printCompressedLimitedString(WINDOW *win, int y, int x, string str_in, unsigned long limit) {

  std::string chars("aeiouAEIOU :");

  for (size_t i = 0; i < chars.length(); i++) {
    str_in.erase(std::remove(str_in.begin() + 1, str_in.end(), chars.at(i)), str_in.end());
  }

  if (str_in.length() > limit) {
    str_in.resize(limit);
  }

  const char *format = str_in.c_str();

  mvwprintw(win, y, x, format);
}

//}

/* printNoData() //{ */

void Status::printNoData(WINDOW *win, int y, int x) {

  wattron(win, A_BLINK);
  wattron(win, COLOR_PAIR(RED));
  if (mini_) {
    mvwprintw(win, y, x, "NO DATA");
  } else {
    mvwprintw(win, y, x, "!NO DATA!");
  }
  wattroff(win, COLOR_PAIR(RED));
  wattroff(win, A_BLINK);
}

void Status::printNoData(WINDOW *win, int y, int x, string text) {

  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, text.c_str());
  printNoData(win, y, x + text.length());
}

//}

/* printError() //{ */

void Status::printError(string msg) {

  wattron(debug_window_, COLOR_PAIR(RED));
  printLimitedString(debug_window_, 0, 0, msg, 120);
  wattroff(debug_window_, COLOR_PAIR(RED));

  wnoutrefresh(debug_window_);
}

//}

/* printDebug() //{ */

void Status::printDebug(string msg) {

  printLimitedString(debug_window_, 0, 0, msg, 120);

  wnoutrefresh(debug_window_);
}

//}

/* printHelp() //{ */

void Status::printHelp() {

  werase(debug_window_);

  if (help_active_) {

    printLimitedString(debug_window_, 1, 0, "How to use mrs_status:", 120);
    printLimitedString(debug_window_, 2, 0, "Press the 'm' key to enter a services menu", 120);
    printLimitedString(debug_window_, 3, 0, "Press the 'g' key to set a goto reference", 120);
    printLimitedString(debug_window_, 4, 0, "Press the 'M' key to switch into minimalistic mode, which takes less screen space", 120);
    printLimitedString(debug_window_, 5, 0, "Press the 'R' key to enter 'remote' mode to take direct control of the uav with your keyboad", 120);
    printLimitedString(debug_window_, 6, 0, "   In remote mode, use these keys to control the drone:", 120);
    printLimitedString(debug_window_, 7, 0, "      'w','s','a','d' to control pitch and roll ('h','j','k','l' works too)", 120);
    printLimitedString(debug_window_, 8, 0, "      'q','e'         to control heading", 120);
    printLimitedString(debug_window_, 9, 0, "      'r','f'         to control altitude", 120);
    printLimitedString(debug_window_, 10, 0, "      'G'             to switch controlling in the FCU frame (local) or the world frame (global)", 120);
    printLimitedString(debug_window_, 12, 0, "You can also display any info from your node in the mrs_status:", 120);
    printLimitedString(debug_window_, 14, 0, "   topic: mrs_status/display_string (std_msgs::String)", 120);
    printLimitedString(debug_window_, 15, 0, "   - Publish any string to this topic and it will show up in mrs_status", 120);
    printLimitedString(debug_window_, 17, 0, "Press 'D' to display info from other panes of this tmux session, up to 2 panes can be viewed", 120);
    printLimitedString(debug_window_, 19, 0, "Press 'h' to hide help", 120);

  } else {
    printLimitedString(debug_window_, 1, 0, "Press 'h' key for help", 120);
  }

  wnoutrefresh(debug_window_);
}

//}

/* printTmuxDump() //{ */

void Status::printTmuxDump() {

  werase(debug_window_);
  printBox(debug_window_);

  if (int(selected_tmux_window_.size()) > MAX_SELECTED_TMUX_WINDOWS) {
    return;
  }

  int tmp_cols, tmp_rows;
  getmaxyx(sub_tmux_window_1_, tmp_rows, tmp_cols);

  for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
    std::string command_str = "tmux resize-window -t " + session_name_ + ":" + std::to_string(selected_tmux_window_[i]) + " -A";
    callTerminal(command_str.c_str());
    command_str = "tmux capture-pane -pt " + session_name_ + ":" + std::to_string(selected_tmux_window_[i]) + " -S 0 | tail -n " + std::to_string(tmp_rows + 1);
    std::string response = callTerminal(command_str.c_str());
    switch (i) {
    case 0: {
      mvwaddstr(sub_tmux_window_1_, 0, 0, response.c_str());
      break;
    }
    case 1: {
      mvwaddstr(sub_tmux_window_2_, 0, 0, response.c_str());
      break;
    }
    }
  }

  mvwhline(debug_window_, tmp_rows + 1, 1, 0, tmp_cols - 1);

  if (selected_tmux_window_.size() > 1) {
    printLimitedString(debug_window_, tmp_rows + 1, 3, display_menu_text_[selected_tmux_window_[1]], 50);
  }
  if (selected_tmux_window_.size() > 0) {
    printLimitedString(debug_window_, 0, 3, display_menu_text_[selected_tmux_window_[0]], 50);
  }

  wnoutrefresh(debug_window_);
  touchwin(debug_window_);

  wnoutrefresh(sub_tmux_window_1_);
  wnoutrefresh(sub_tmux_window_2_);
}

//}

/* printBox() //{ */

void Status::printBox(WINDOW *win) {

  if (avoiding_collision_) {

    wattron(win, COLOR_PAIR(RED));
    wattron(win, A_BLINK);
    wattron(win, A_STANDOUT);
  }

  if (!automatic_start_can_takeoff_ && null_tracker_) {

    wattron(win, COLOR_PAIR(YELLOW));
    wattron(win, A_STANDOUT);
  }


  box(win, 0, 0);
  wattroff(win, A_BLINK);
  wattroff(win, COLOR_PAIR(RED));
  wattroff(win, A_STANDOUT);
}

//}

/* setupColors() //{ */

void Status::setupColors(bool active) {

  init_pair(ALWAYS_RED, COLOR_NICE_RED, BACKGROUND_DEFAULT);

  if (active) {

    init_pair(NORMAL, COLOR_WHITE, BACKGROUND_DEFAULT);
    init_pair(FIELD, COLOR_WHITE, 235);
    init_pair(RED, COLOR_NICE_RED, BACKGROUND_DEFAULT);
    init_pair(YELLOW, COLOR_NICE_YELLOW, BACKGROUND_DEFAULT);

    if (_colorblind_mode_) {
      init_pair(GREEN, COLOR_NICE_BLUE, BACKGROUND_DEFAULT);
    } else {
      init_pair(GREEN, COLOR_NICE_GREEN, BACKGROUND_DEFAULT);
    }
    _light_ = false;


    if (_colorscheme_.find("COLORSCHEME_LIGHT") != std::string::npos) {
      init_pair(NORMAL, COLOR_BLACK, BACKGROUND_DEFAULT);
      init_pair(FIELD, COLOR_WHITE, 237);
      init_pair(YELLOW, COLOR_DARK_YELLOW, BACKGROUND_DEFAULT);
      if (_colorblind_mode_) {
        init_pair(GREEN, COLOR_DARK_BLUE, BACKGROUND_DEFAULT);
      } else {
        init_pair(GREEN, COLOR_DARK_GREEN, BACKGROUND_DEFAULT);
      }
      _light_ = true;
    }

  } else {
    init_pair(NORMAL, COLOR_DARK_RED, BACKGROUND_DEFAULT);
    init_pair(FIELD, COLOR_DARK_RED, 235);
    init_pair(RED, COLOR_DARK_RED, BACKGROUND_DEFAULT);
    init_pair(YELLOW, COLOR_DARK_RED, BACKGROUND_DEFAULT);
    init_pair(GREEN, COLOR_DARK_RED, BACKGROUND_DEFAULT);
    _light_ = false;


    if (_colorscheme_.find("COLORSCHEME_LIGHT") != std::string::npos) {
      init_pair(FIELD, COLOR_DARK_RED, 237);
      _light_ = true;
    }
  }
}

//}

//}

/* callTerminal() //{ */

std::string Status::callTerminal(const char *cmd) {

  std::array<char, 128>                    buffer;
  std::string                              result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

  if (!pipe) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in callTerminal");
    throw std::runtime_error("popen() failed!");
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
}

//}

} // namespace mrs_uav_status

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<mrs_uav_status::Status>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

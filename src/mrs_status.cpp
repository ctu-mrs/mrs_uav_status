
/* INCLUDES //{ */

#include <status_window.h>
/* #include <commons.h> */

#include <stdlib.h>
#include <stdio.h>
#include <mutex>

#include <fstream>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/BatteryState.h>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

#include <mrs_lib/transformer.h>
#include <mrs_lib/ParamLoader.h>


using namespace std;
using topic_tools::ShapeShifter;

//}

class MrsStatus;

/* MENU CLASS //{ */

/* ------------------- MENU CLASS ------------------- */

#define KEY_ENT 10
#define KEY_ESC 27

/* class Menu //{ */

class Menu {

public:
  Menu(std::shared_ptr<WINDOW> win);
  Menu(std::shared_ptr<WINDOW> win, std::vector<string> menu_items);
  void Activate();

private:
  std::shared_ptr<WINDOW> win;
  std::vector<string>     menu_items;
};

//}

/* Menu() //{ */

Menu::Menu(std::shared_ptr<WINDOW> win) {

  this->win = win;  // set the pointer to the window that this menu is contained in
}

Menu::Menu(std::shared_ptr<WINDOW> win, std::vector<string> menu_items) {

  this->win        = win;         // set the pointer to the window that this menu is contained in
  this->menu_items = menu_items;  // set the pointer to the window that this menu is contained in
}

void Menu::Activate() {

  box(win.get(), '|', '-');

  for (unsigned long i = 0; i < menu_items.size(); i++) {

    if (i == 0)

      wattron(win.get(), A_STANDOUT);  // highlights the first item.

    else

      wattroff(win.get(), A_STANDOUT);

    mvwaddstr(win.get(), i + 1, 2, menu_items[i].c_str());
  }

  wrefresh(win.get());  // update the terminal screen

  keypad(win.get(), TRUE);  // enable keyboard input for the window.
  curs_set(0);              // hide the default screen cursor.

  int  ch  = 0;
  int  i   = 0;
  bool run = true;

  while (run) {
    ch = wgetch(win.get());
    // right pad with spaces to make the items appear with even width.

    mvwaddstr(win.get(), i + 1, 2, menu_items[i].c_str());
    // use a variable to increment or decrement the value based on the input.
    if (ch == KEY_UP || ch == 'k') {
      i--;
      i = (i < 0) ? menu_items.size() - 1 : i;
    } else if (ch == KEY_DOWN || ch == 'j') {
      i++;
      i = (i > int(menu_items.size() - 1)) ? 0 : i;
    } else if (ch == KEY_ENT) {
      menu_items[i] = "e";
    } else if (ch == 'q' || ch == KEY_ESC) {
      run = false;
    }

    // now highlight the next item in the list.
    //
    wattron(win.get(), A_STANDOUT);
    mvwaddstr(win.get(), i + 1, 2, menu_items[i].c_str());
    wattroff(win.get(), A_STANDOUT);
  }
}

//}

/* ----------------- //MENU CLASS ------------------- */

//}

/* class MrsStatus //{ */

class MrsStatus {

public:
  MrsStatus();

private:
  ros::NodeHandle nh_;

  ros::Timer status_timer_;

  void UavStateCallback(const mrs_msgs::UavStateConstPtr& msg);
  void MavrosStateCallback(const mavros_msgs::StateConstPtr& msg);
  void MavrosAttitudeCallback(const mavros_msgs::AttitudeTargetConstPtr& msg);
  void BatteryCallback(const sensor_msgs::BatteryStateConstPtr& msg);
  void ControlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  void GainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg);
  void ConstraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg);

  void GenericCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name, const int id);

  void statusTimer(const ros::TimerEvent& event);

  void PrintLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit);
  void PrintLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit);

  void PrintNoData(WINDOW* win, int y, int x);
  void PrintNoData(WINDOW* win, int y, int x, string text);

  void PrintCpuLoad(WINDOW* win);
  void PrintCpuFreq(WINDOW* win);
  void PrintMemLoad(WINDOW* win);

  void GenericTopicHandler(WINDOW* win, double rate, short color, int topic);
  void UavStateHandler(WINDOW* win, double rate, short color, int topic);
  void MavrosStateHandler(WINDOW* win, double rate, short color, int topic);
  void ControlManagerHandler(WINDOW* win, double rate, short color, int topic);
  void GeneralInfoHandler(WINDOW* win, double rate, short color, int topic);

  ros::Subscriber uav_state_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;

  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber mavros_attitude_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Subscriber control_manager_subscriber_;
  ros::Subscriber gain_manager_subscriber_;
  ros::Subscriber constraint_manager_subscriber_;

  std::string _uav_name_;
  std::string _uav_type_;
  std::string _sensors_;
  bool        _pixgarm_;

  bool initialized_ = false;

  long last_idle_  = 0;
  long last_total_ = 0;

  mrs_msgs::UavState uav_state_;
  /* std::shared_ptr<int> uav_state_counter_ptr_ = std::make_shared<int>(0); */

  mavros_msgs::State          mavros_state_;
  mavros_msgs::AttitudeTarget mavros_attitude_;
  sensor_msgs::BatteryState   battery_;

  mrs_msgs::ControlManagerDiagnostics    control_manager_;
  mrs_msgs::GainManagerDiagnostics       gain_manager_;
  mrs_msgs::ConstraintManagerDiagnostics constraint_manager_;

  StatusWindow*      uav_state_window_;
  std::vector<topic> uav_state_topics_;

  StatusWindow*      mavros_state_window_;
  std::vector<topic> mavros_state_topics_;

  StatusWindow*      control_manager_window_;
  std::vector<topic> control_manager_topics_;

  StatusWindow*      general_info_window_;
  std::vector<topic> general_info_topics_;

  StatusWindow*                generic_topics_window_;
  std::vector<topic>           generic_topics_vec_;
  std::vector<string>          generic_topics_input_vec_;
  std::vector<ros::Subscriber> generic_subscriber_vec_;
};

//}

/* MrsStatus() //{ */

MrsStatus::MrsStatus() {

  // initialize node and create no handle
  nh_ = ros::NodeHandle("~");

  // TIMERS
  status_timer_ = nh_.createTimer(ros::Rate(10), &MrsStatus::statusTimer, this);

  // SUBSCRIBERS
  uav_state_subscriber_          = nh_.subscribe("uav_state_in", 1, &MrsStatus::UavStateCallback, this, ros::TransportHints().tcpNoDelay());
  mavros_state_subscriber_       = nh_.subscribe("mavros_state_in", 1, &MrsStatus::MavrosStateCallback, this, ros::TransportHints().tcpNoDelay());
  mavros_attitude_subscriber_    = nh_.subscribe("mavros_attitude_in", 1, &MrsStatus::MavrosAttitudeCallback, this, ros::TransportHints().tcpNoDelay());
  battery_subscriber_            = nh_.subscribe("battery_in", 1, &MrsStatus::BatteryCallback, this, ros::TransportHints().tcpNoDelay());
  control_manager_subscriber_    = nh_.subscribe("control_manager_in", 1, &MrsStatus::ControlManagerCallback, this, ros::TransportHints().tcpNoDelay());
  gain_manager_subscriber_       = nh_.subscribe("gain_manager_in", 1, &MrsStatus::GainManagerCallback, this, ros::TransportHints().tcpNoDelay());
  constraint_manager_subscriber_ = nh_.subscribe("constraint_manager_in", 1, &MrsStatus::ConstraintManagerCallback, this, ros::TransportHints().tcpNoDelay());

  mrs_lib::ParamLoader param_loader(nh_, "MrsStatus");

  param_loader.load_param("uav_name", _uav_name_);
  param_loader.load_param("uav_type", _uav_type_);
  param_loader.load_param("sensors", _sensors_);
  param_loader.load_param("pixgarm", _pixgarm_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[LidarFlier]: Could not load all parameters!");
    ros::shutdown();
  } else {
    ROS_INFO("[LidarFlier]: All params loaded!");
  }


  std::vector<std::string> results;
  split(results, _sensors_, boost::is_any_of(", "), boost::token_compress_on);

  std::vector<string> generic_topics_input_vec_;

  if (_pixgarm_) {
    generic_topics_input_vec_.push_back("mavros/distance_sensor/garmin Garmin_pix 80+");
  }

  for (int i = 0; i < results.size(); i++) {
    if (results[i] == "garmin_down" && _pixgarm_ == false) {
      generic_topics_input_vec_.push_back("garmin/range Garmin_Down 80+");

    } else if (results[i] == "garmin_up") {
      generic_topics_input_vec_.push_back("garmin/range_up Garmin_Up 80+");

    } else if (results[i] == "realsense_brick") {
      generic_topics_input_vec_.push_back("rs_d435/depth/camera_info Realsense_Brick 25+");

    } else if (results[i] == "realsense_front") {
      generic_topics_input_vec_.push_back("rs_d435/depth/camera_info Realsense_Front 25+");

    } else if (results[i] == "bluefox_brick") {
      generic_topics_input_vec_.push_back("bluefox_brick/camera_info Bluefox_Brick 25+");

    } else if (results[i] == "bluefox_optflow") {
      generic_topics_input_vec_.push_back("bluefox_optflow/camera_info Bluefox_Optflow 60+");
      generic_topics_input_vec_.push_back("optic_flow/velocity Optic_flow 60+");

    } else if (results[i] == "trinocular_thermal") {
      generic_topics_input_vec_.push_back("thermal/top/rgb_image Thermal_Top 15+");
      generic_topics_input_vec_.push_back("thermal/middle/rgb_image Thermal_Middle 15+");
      generic_topics_input_vec_.push_back("thermal/bottom/rgb_image Thermal_Bottom 15+");

    } else if (results[i] == "rplidar") {
      generic_topics_input_vec_.push_back("rplidar/scan Rplidar 10+");
    }

    /* } else if (results[i] == "rplidar") { */
    /*   generic_topics_input_vec_.push_back("rplidar/scan Rplidar 10+"); */
    /* } */


    /* if str(self.PIXGARM) == "true": */
    /*             self.param_list.insert(0, "mavros/distance_sensor/garmin Garmin_pix 80+") */

    /*         if str(self.PIXGARM) == "false" and 'garmin_down' in self.sensor_list: */
    /*             self.param_list.insert(0, "garmin/range Garmin_Down 80+") */


    /*         if str(self.BLUEFOX_UV_LEFT) != "": */
    /*             self.param_list.insert(0, "uvdar_bluefox/left/camera_info Bluefox_UV_left 70+") */

    /*         if str(self.BLUEFOX_UV_RIGHT) != "": */
    /*             self.param_list.insert(0, "uvdar_bluefox/right/camera_info Bluefox_UV_right 70+") */

    /*         if str(self.ODOMETRY_TYPE) == "gps": */
    /*             self.param_list.insert(0, "mavros/global_position/global PX4 GPS 100") */
  }

  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;  // generic callback

  for (unsigned long i = 0; i < generic_topics_input_vec_.size(); i++) {

    std::vector<std::string> results;
    boost::split(results, generic_topics_input_vec_[i], [](char c) { return c == ' '; });  // split the input string into words and put them in results vector
    if (results[2].back() == '+') {
      // TODO handle the + sign
      results[2].pop_back();
    }

    topic tmp_topic(results[0], results[1], std::stoi(results[2]));

    generic_topics_vec_.push_back(tmp_topic);

    int    id         = i;  // id to identify which topic called the generic callback
    string topic_name = "/" + _uav_name_ + "/" + generic_topics_vec_[i].topic_name;

    callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { GenericCallback(msg, topic_name, id); };
    ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 1, callback);

    generic_subscriber_vec_.push_back(tmp_subscriber);
  }

  uav_state_topics_.push_back(topic{100.0});

  uav_state_window_ = new StatusWindow(6, 30, 5, 1, uav_state_topics_);

  control_manager_topics_.push_back(topic{10.0});
  control_manager_topics_.push_back(topic{1.0});
  control_manager_topics_.push_back(topic{1.0});

  control_manager_window_ = new StatusWindow(4, 30, 1, 1, control_manager_topics_);

  mavros_state_topics_.push_back(topic{100.0});
  mavros_state_topics_.push_back(topic{1.0});
  mavros_state_topics_.push_back(topic{100.0});

  mavros_state_window_ = new StatusWindow(6, 30, 5, 31, mavros_state_topics_);

  general_info_window_ = new StatusWindow(4, 30, 1, 31, general_info_topics_);

  generic_topics_window_ = new StatusWindow(10, 30, 1, 61, generic_topics_vec_);

  initialized_ = true;
  ROS_INFO("[Mrs Status]: Node initialized!");

  /* refresh(); */
}

//}

/* statusTimer //{ */

void MrsStatus::statusTimer([[maybe_unused]] const ros::TimerEvent& event) {

  uav_state_window_->Redraw(&MrsStatus::UavStateHandler, this);
  mavros_state_window_->Redraw(&MrsStatus::MavrosStateHandler, this);
  control_manager_window_->Redraw(&MrsStatus::ControlManagerHandler, this);
  generic_topics_window_->Redraw(&MrsStatus::GenericTopicHandler, this);
  general_info_window_->Redraw(&MrsStatus::GeneralInfoHandler, this);
  refresh();
}

//}

/* GeneralInfoHandler() //{ */

void MrsStatus::GeneralInfoHandler(WINDOW* win, double rate, short color, int topic) {

  PrintCpuLoad(win);
  PrintMemLoad(win);
  PrintCpuFreq(win);
}

//}

/* GenericTopicHandler() //{ */

void MrsStatus::GenericTopicHandler(WINDOW* win, double rate, short color, int topic) {

  PrintLimitedString(win, 1 + topic, 1, generic_topics_vec_[topic].topic_display_name, 19);
  PrintLimitedDouble(win, 1 + topic, 21, "%5.1f Hz", rate, 1000);
}

//}

/* UavStateHandler() //{ */

void MrsStatus::UavStateHandler(WINDOW* win, double rate, short color, int topic) {

  PrintLimitedDouble(win, 0, 16, "Odom %5.1f Hz", rate, 1000);

  if (rate == 0) {

    PrintNoData(win, 0, 1);

  } else {

    double         roll, pitch, yaw;
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(uav_state_.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(roll, pitch, yaw);

    PrintLimitedDouble(win, 1, 2, "X %7.2f", uav_state_.pose.position.x, 1000);
    PrintLimitedDouble(win, 2, 2, "Y %7.2f", uav_state_.pose.position.y, 1000);
    PrintLimitedDouble(win, 3, 2, "Z %7.2f", uav_state_.pose.position.z, 1000);
    PrintLimitedDouble(win, 4, 2, "Yaw %5.2f", yaw, 1000);

    int pos = uav_state_.header.frame_id.find("/") + 1;
    PrintLimitedString(win, 1, 14, uav_state_.header.frame_id.substr(pos, uav_state_.header.frame_id.length()), 15);

    PrintLimitedString(win, 2, 14, "Hori: " + uav_state_.estimator_horizontal.name, 15);
    PrintLimitedString(win, 3, 14, "Vert: " + uav_state_.estimator_vertical.name, 15);
    PrintLimitedString(win, 4, 14, "Head: " + uav_state_.estimator_heading.name, 15);
  }
}

//}

/* MavrosStateHandler() //{ */

void MrsStatus::MavrosStateHandler(WINDOW* win, double rate, short color, int topic) {

  string tmp_string;

  switch (topic) {
    case 0:  // mavros state
      PrintLimitedDouble(win, 0, 14, "Mavros %5.1f Hz", rate, 1000);

      if (rate == 0) {

        PrintNoData(win, 0, 1);

      } else {

        if (mavros_state_.armed) {
          tmp_string = "ARMED";
        } else {
          tmp_string = "DISARMED";
          wattron(win, COLOR_PAIR(RED));
        }

        PrintLimitedString(win, 1, 1, "State: " + tmp_string, 15);
        wattron(win, COLOR_PAIR(color));

        if (mavros_state_.mode != "OFFBOARD") {
          wattron(win, COLOR_PAIR(RED));
        }

        PrintLimitedString(win, 2, 1, "Mode:  " + mavros_state_.mode, 15);
        wattron(win, COLOR_PAIR(color));
      }

      break;

    case 1:  // battery
      if (rate == 0) {

        PrintNoData(win, 3, 1, "Batt:  ");

      } else {

        double voltage = battery_.voltage;
        (voltage > 17.0) ? (voltage = voltage / 6) : (voltage = voltage / 4);

        if (voltage < 3.6) {
          wattron(win, COLOR_PAIR(RED));
        } else if (voltage < 3.7 && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        }

        PrintLimitedDouble(win, 3, 1, "Batt:  %4.2f V", voltage, 10);
        wattron(win, COLOR_PAIR(color));

        PrintLimitedDouble(win, 3, 15, "%5.2f A", battery_.current, 100);
      }
      break;

    case 2:  // mavros attitude
      if (rate == 0) {

        PrintNoData(win, 4, 1, "Thrst: ");

      } else {

        if (mavros_attitude_.thrust > 0.75) {
          wattron(win, COLOR_PAIR(RED));
        } else if (mavros_attitude_.thrust > 0.65 && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        }
        PrintLimitedDouble(win, 4, 1, "Thrst: %4.2f", mavros_attitude_.thrust, 1.01);
        wattron(win, COLOR_PAIR(color));
      }
      break;
  }
}

//}

/* ControlManagerHandler() //{ */

void MrsStatus::ControlManagerHandler(WINDOW* win, double rate, short color, int topic) {

  string controller;
  string tracker;
  controller = control_manager_.controller_status.controller;
  tracker    = control_manager_.tracker_status.tracker;

  switch (topic) {
    case 0:  // mavros state
      PrintLimitedString(win, 0, 14, "Control Manager", 15);

      if (rate == 0) {

        PrintNoData(win, 0, 1);

        wattron(win, COLOR_PAIR(RED));
        mvwprintw(win, 1, 1, "NO_CONTROLLER");
        mvwprintw(win, 2, 1, "NO_TRACKER");
        wattron(win, COLOR_PAIR(color));

      } else {
        if (controller != "So3Controller") {
          if (controller != "MpcController") {
            wattron(win, COLOR_PAIR(RED));
          }
          PrintLimitedString(win, 1, 1, controller, 29);
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

          PrintLimitedString(win, 2, 1, tracker, 29);

        } else {
          mvwprintw(win, 2, 1, tracker.c_str());
          wattron(win, COLOR_PAIR(NORMAL));
          mvwprintw(win, 2, 1 + tracker.length(), "%s", "/");
          wattron(win, COLOR_PAIR(color));
        }
      }
      break;

    case 1:  // mavros state

      if (controller == "So3Controller") {

        if (rate == 0) {
          PrintNoData(win, 1, 2 + controller.length());
        } else {
          PrintLimitedString(win, 1, 2 + controller.length(), gain_manager_.current_name, 10);
        }
      }
      break;

    case 2:  // mavros state

      if (tracker == "MpcTracker") {
        if (rate == 0) {
          PrintNoData(win, 2, 2 + tracker.length());
        } else {
          PrintLimitedString(win, 2, 2 + tracker.length(), constraint_manager_.current_name, 10);
        }
      }
      break;
  }
}

//}

/* PrintMemLoad() //{ */

void MrsStatus::PrintMemLoad(WINDOW* win) {

  std::ifstream fileStat("/proc/meminfo");
  std::string   line1, line2, line3, line4;
  std::getline(fileStat, line1);
  std::getline(fileStat, line2);
  std::getline(fileStat, line3);
  std::getline(fileStat, line4);

  std::vector<std::string> results;
  boost::split(results, line1, [](char c) { return c == ' '; });

  double ram_total;
  double ram_free;
  double ram_used;
  double buffers;

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        ram_total = double(std::stol(results[i])) / 1000000;
      }
      catch (const std::invalid_argument& e) {
        ram_total = 0.0;
      }
      break;
    }
  }

  boost::split(results, line3, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        ram_free = double(std::stol(results[i])) / 1000000;
      }
      catch (const std::invalid_argument& e) {
        ram_free = 0.0;
      }
      break;
    }
  }

  boost::split(results, line4, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        buffers = double(std::stol(results[i])) / 1000000;
      }
      catch (const std::invalid_argument& e) {
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

  PrintLimitedDouble(win, 2, 1, "RAM free: %4.1f G", (ram_free+buffers), 100);

  /* wattron(win, COLOR_PAIR(NORMAL)); */

  /* PrintLimitedDouble(win, 2, 15, "/%4.1f", ram_total, 100); */
}

//}

/* PrintCpuLoad() //{ */

void MrsStatus::PrintCpuLoad(WINDOW* win) {

  std::ifstream fileStat("/proc/stat");
  std::string   line;
  std::getline(fileStat, line);

  std::vector<std::string> results;
  boost::split(results, line, [](char c) { return c == ' '; });

  long idle;
  long non_idle;
  long total;

  try {
    idle     = std::stol(results[5]) + std::stol(results[6]);
    non_idle = std::stol(results[2]) + std::stol(results[3]) + std::stol(results[4]) + std::stol(results[7]) + std::stol(results[8]) + std::stol(results[9]);
    total    = idle + non_idle;
  }
  catch (const std::invalid_argument& e) {
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

  PrintLimitedDouble(win, 1, 1, "CPU load: %4.1f %%", cpu_load, 99.9);
}

//}

/* PrintCpuFreq() //{ */

void MrsStatus::PrintCpuFreq(WINDOW* win) {

  std::ifstream fileStat("/sys/devices/system/cpu/online");
  std::string   line;
  std::getline(fileStat, line);

  std::vector<std::string> results;
  boost::split(results, line, [](char c) { return c == '-'; });

  int num_cores;

  try {
    num_cores = std::stoi(results[1]) + 1;
  }
  catch (const std::invalid_argument& e) {
    num_cores = 0;
  }

  long cpu_freq = 0;

  for (int i = 0; i < num_cores; i++) {
    std::string   filename = "/sys/devices/system/cpu/cpu" + to_string(i) + "/cpufreq/scaling_cur_freq";
    std::ifstream fileStat(filename.c_str());
    std::getline(fileStat, line);
    try {
      cpu_freq += std::stol(line);
    }
    catch (const std::invalid_argument& e) {
      cpu_freq = 0;
    }
  }

  double avg_cpu_ghz = double(cpu_freq / num_cores)/1000000;

  wattron(win, COLOR_PAIR(GREEN));
  PrintLimitedDouble(win, 1, 21, "%4.2f GHz", avg_cpu_ghz, 10);
}

//}

/* PrintLimitedDouble() //{ */

void MrsStatus::PrintLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit) {

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

/* PrintLimitedString() //{ */

void MrsStatus::PrintLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit) {

  if (str_in.length() > limit) {
    str_in.resize(limit);
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format);
}

//}

/* PrintNoData() //{ */

void MrsStatus::PrintNoData(WINDOW* win, int y, int x) {

  wattron(win, A_BLINK);
  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, "!NO DATA!");
  wattroff(win, COLOR_PAIR(RED));
  wattroff(win, A_BLINK);
}

//}

/* PrintNoData() //{ */

void MrsStatus::PrintNoData(WINDOW* win, int y, int x, string text) {

  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, text.c_str());
  PrintNoData(win, y, x + text.length());
}

//}

/* UavStateCallback() //{ */

void MrsStatus::UavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {
  uav_state_topics_[0].counter++;
  uav_state_ = *msg;
}

//}

/* MavrosStateCallback() //{ */

void MrsStatus::MavrosStateCallback(const mavros_msgs::StateConstPtr& msg) {
  mavros_state_topics_[0].counter++;
  mavros_state_ = *msg;
}

//}

/* BatteryCallback() //{ */

void MrsStatus::BatteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
  mavros_state_topics_[1].counter++;
  battery_ = *msg;
}

//}

/* MavrosAttitudeCallback() //{ */

void MrsStatus::MavrosAttitudeCallback(const mavros_msgs::AttitudeTargetConstPtr& msg) {
  mavros_state_topics_[2].counter++;
  mavros_attitude_ = *msg;
}

//}

/* ControlManagerCallback() //{ */

void MrsStatus::ControlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {
  control_manager_topics_[0].counter++;
  control_manager_ = *msg;
}

//}

/* GainManagerCallback() //{ */

void MrsStatus::GainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg) {
  control_manager_topics_[1].counter++;
  gain_manager_ = *msg;
}

//}

/* ConstraintManagerCallback() //{ */

void MrsStatus::ConstraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {
  control_manager_topics_[2].counter++;
  constraint_manager_ = *msg;
}

//}

/* GenericCallback() //{ */

void MrsStatus::GenericCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name, const int id) {
  generic_topics_vec_[id].counter++;
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
  curs_set(0);  // disable cursor
  use_default_colors();

  init_pair(NORMAL, COLOR_WHITE, BACKGROUND_DEFAULT);
  init_pair(RED, COLOR_NICE_RED, BACKGROUND_DEFAULT);
  init_pair(YELLOW, COLOR_NICE_YELLOW, BACKGROUND_DEFAULT);
  init_pair(GREEN, COLOR_NICE_GREEN, BACKGROUND_DEFAULT);
  init_pair(BLUE, COLOR_NICE_BLUE, BACKGROUND_DEFAULT);

  attron(A_BOLD);

  /* DEBUG COLOR RAINBOW //{ */

  /* for (int j = 0; j < 256; j++) { */
  /*   init_pair(j, j, BACKGROUND_DEFAULT); */
  /* } */

  /* int k = 0; */
  /* for (int j = 0; j < 256; j++) { */
  /*   attron(COLOR_PAIR(j)); */
  /*   mvwprintw(stdscr, k, (3 * j + 1) - k * 60 * 3, "%i", j); */
  /*   k = int(j / 60); */
  /*   attroff(COLOR_PAIR(j)); */
  /* } */

  //}

  MrsStatus status;

  /* std::shared_ptr<WINDOW> menu_win(newwin(10, 12, 1, 1)); */
  /* Menu menu(menu_win, items); */
  /* menu.Activate(); */
  /* endwin(); */

  while (ros::ok()) {

    ros::spin();
    return 0;
  }
}

//

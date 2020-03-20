
/* INCLUDES //{ */

#include <status_window.h>
/* #include <commons.h> */

#include <stdlib.h>
#include <stdio.h>
#include <mutex>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <std_srvs/Trigger.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/BatteryState.h>

#include <boost/function.hpp>

#include <mrs_lib/transformer.h>


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

  void GenericCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name);

  void statusTimer(const ros::TimerEvent& event);

  void PrintLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit);
  void PrintLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit);

  void UavStateHandler(WINDOW* win, double rate, short color, int topic);
  void MavrosStateHandler(WINDOW* win, double rate, short color, int topic);
  void ControlManagerHandler(WINDOW* win, double rate, short color, int topic);

  ros::Subscriber uav_state_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;

  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber mavros_attitude_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Subscriber control_manager_subscriber_;
  ros::Subscriber gain_manager_subscriber_;
  ros::Subscriber constraint_manager_subscriber_;

  ros::Subscriber generic_subscriber;

  std::string _uav_name_;
  int         counter = 0;

  bool initialized_ = false;

  mrs_msgs::UavState   uav_state_;
  std::shared_ptr<int> uav_state_counter_ptr_ = std::make_shared<int>(0);

  mavros_msgs::State          mavros_state_;
  mavros_msgs::AttitudeTarget mavros_attitude_;
  sensor_msgs::BatteryState   battery_;
  std::shared_ptr<int> mavros_state_counter_ptr_ = std::make_shared<int>(0);
  std::shared_ptr<int> mavros_attitude_counter_ptr_ = std::make_shared<int>(0);
  std::shared_ptr<int> battery_counter_ptr_ = std::make_shared<int>(0);

  mrs_msgs::ControlManagerDiagnostics    control_manager_;
  mrs_msgs::GainManagerDiagnostics       gain_manager_;
  mrs_msgs::ConstraintManagerDiagnostics constraint_manager_;
  std::shared_ptr<int> control_manager_counter_ptr_ = std::make_shared<int>(0);
  std::shared_ptr<int> gain_manager_counter_ptr_ = std::make_shared<int>(0);
  std::shared_ptr<int> constraint_manager_counter_ptr_ = std::make_shared<int>(0);

  StatusWindow*           uav_state_window;
  std::vector<topic_rate> uav_state_rates_;

  StatusWindow*           mavros_state_window;
  std::vector<topic_rate> mavros_state_rates_;

  StatusWindow*           control_manager_window;
  std::vector<topic_rate> control_manager_rates_;
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

  string topic_name = "/uav1/odometry/odom_main";

  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
  callback           = [this, topic_name](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { GenericCallback(msg, topic_name); };
  generic_subscriber = nh_.subscribe(topic_name, 10, callback);

  uav_state_rates_.push_back(topic_rate{uav_state_counter_ptr_, 0.0, 100.0});

  uav_state_window = new StatusWindow(6, 30, 3, 3, uav_state_rates_);

  mavros_state_rates_.push_back(topic_rate{mavros_state_counter_ptr_, 0.0, 100.0});
  mavros_state_rates_.push_back(topic_rate{mavros_attitude_counter_ptr_, 0.0, 100.0});
  mavros_state_rates_.push_back(topic_rate{battery_counter_ptr_, 0.0, 1.0});

  mavros_state_window = new StatusWindow(6, 30, 9, 3, mavros_state_rates_);

  control_manager_rates_.push_back(topic_rate{control_manager_counter_ptr_, 0.0, 10.0});
  control_manager_rates_.push_back(topic_rate{gain_manager_counter_ptr_, 0.0, 1.0});
  control_manager_rates_.push_back(topic_rate{constraint_manager_counter_ptr_, 0.0, 1.0});

  control_manager_window = new StatusWindow(4, 30, 16, 3, control_manager_rates_);

  initialized_ = true;
  ROS_INFO("[Mrs Status]: Node initialized!");

  /* refresh(); */
}

//}

/* statusTimer //{ */

void MrsStatus::statusTimer([[maybe_unused]] const ros::TimerEvent& event) {

  uav_state_window->Redraw(&MrsStatus::UavStateHandler, this);
  mavros_state_window->Redraw(&MrsStatus::MavrosStateHandler, this);
  control_manager_window->Redraw(&MrsStatus::ControlManagerHandler, this);
  refresh();
}

//}

/* UavStateHandler() //{ */

void MrsStatus::UavStateHandler(WINDOW* win, double rate, short color, int topic) {
  if (topic != 0) {
    return;
  }

  double         roll, pitch, yaw;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(uav_state_.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

  PrintLimitedDouble(win, 0, 16, "Odom %5.1f Hz", rate, 1000);
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

//}

/* MavrosStateHandler() //{ */

void MrsStatus::MavrosStateHandler(WINDOW* win, double rate, short color, int topic) {

  if (topic != 0) {
    return;
  }
  PrintLimitedDouble(win, 0, 14, "Mavros %5.1f Hz", rate, 1000);

  string tmp_string;
  /* mavros_state_.armed ? tmp_string = "ARMED" : tmp_string = "DISARMED"; */

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


  if (mavros_attitude_.thrust > 0.75) {
    wattron(win, COLOR_PAIR(RED));
  } else if (mavros_attitude_.thrust > 0.65 && color != RED) {
    wattron(win, COLOR_PAIR(YELLOW));
  }
  PrintLimitedDouble(win, 4, 1, "Thrst: %4.2f", mavros_attitude_.thrust, 1.01);
  wattron(win, COLOR_PAIR(color));
}

//}

/* ControlManagerHandler() //{ */

void MrsStatus::ControlManagerHandler(WINDOW* win, double rate, short color, int topic) {

  if (topic != 0) {
    return;
  }

  PrintLimitedString(win, 0, 14, "Control Manager", 15);

  string controller;
  string tracker;
  if (rate > 1.0) {
    controller = control_manager_.controller_status.controller;
    tracker    = control_manager_.tracker_status.tracker;
  } else {
    controller = "NO_CONTROLLER";
    tracker    = "NO_TRACKER";
  }

  if (controller != "So3Controller") {
    if (controller != "MpcController") {
      wattron(win, COLOR_PAIR(RED));
    }
    PrintLimitedString(win, 1, 1, controller, 29);
  } else {
    mvwprintw(win, 1, 1, controller.c_str());
    wattron(win, COLOR_PAIR(NORMAL));
    mvwprintw(win, 1, 1 + controller.length(), "%s", "/");
    wattron(win, COLOR_PAIR(color));
    PrintLimitedString(win, 1, 2 + controller.length(), gain_manager_.current_name, 10);
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
    PrintLimitedString(win, 2, 2 + tracker.length(), constraint_manager_.current_name, 10);
  }

  wattron(win, COLOR_PAIR(color));
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

/* UavStateCallback() //{ */

void MrsStatus::UavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {
  (*uav_state_counter_ptr_)++;
  uav_state_ = *msg;
}

//}

/* MavrosStateCallback() //{ */

void MrsStatus::MavrosStateCallback(const mavros_msgs::StateConstPtr& msg) {
  (*mavros_state_counter_ptr_)++;
  mavros_state_ = *msg;
}

//}

/* MavrosAttitudeCallback() //{ */

void MrsStatus::MavrosAttitudeCallback(const mavros_msgs::AttitudeTargetConstPtr& msg) {
  (*mavros_attitude_counter_ptr_)++;
  mavros_attitude_ = *msg;
}

//}

/* BatteryCallback() //{ */

void MrsStatus::BatteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
  (*battery_counter_ptr_)++;
  battery_ = *msg;
}

//}

/* ControlManagerCallback() //{ */

void MrsStatus::ControlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {
  (*control_manager_counter_ptr_)++;
  control_manager_ = *msg;
}

//}

/* GainManagerCallback() //{ */

void MrsStatus::GainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg) {
  (*gain_manager_counter_ptr_)++;
  gain_manager_ = *msg;
}

//}

/* ConstraintManagerCallback() //{ */

void MrsStatus::ConstraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {
  (*constraint_manager_counter_ptr_)++;
  constraint_manager_ = *msg;
}

//}

/* GenericCallback() //{ */

void MrsStatus::GenericCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name) {
  counter++;
}

//}


/* main() //{ */

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrs_status");

  /* std::vector<string> items; */
  /* items.push_back("1"); */
  /* items.push_back("2"); */
  /* items.push_back("3"); */
  /* items.push_back("4"); */

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

//* INCLUDES //{ */

// some ros includes
#include <ros/ros.h>
#include <ros/package.h>
// some std includes
#include <stdlib.h>
#include <stdio.h>
// mutexes are good to keep your shared variables our of trouble
#include <mutex>

#include <ncurses.h>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/UavState.h>
#include <std_srvs/Trigger.h>
#include <mrs_lib/transformer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using topic_tools::ShapeShifter;

//}

/* MRS_STATUS CLASS //{ */

/* class MrsStatus //{ */

class MrsStatus {

public:
  MrsStatus();

private:
  ros::NodeHandle nh_;

  ros::Timer status_timer_;

  void UavStateCallback(const mrs_msgs::UavStateConstPtr& msg);
  void GenericCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name);

  void statusTimer(const ros::TimerEvent& event);
  void PrintLimitedFloat(WINDOW* win, int y, int x, const char* fmt, ...);

  ros::Subscriber state_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;

  ros::Subscriber generic_subscriber;

  std::string _uav_name_;
  int         counter = 0;

  bool initialized_ = false;


  WINDOW* uav_state_win = newwin(10, 30, 10, 10);

  mrs_msgs::UavState uav_state_;

  int uav_state_counter_ = 0;
};

//}

/* MrsStatus() //{ */

MrsStatus::MrsStatus() {

  // initialize node and create no handle
  nh_ = ros::NodeHandle("~");

  // PARAMS
  /* mrs_lib::ParamLoader param_loader(nh_, "MrsStatus"); */

  /* if (!param_loader.loaded_successfully()) { */
  /*   ROS_ERROR("[MrsStatus]: Could not load all parameters!"); */
  /*   ros::shutdown(); */
  /* } else { */
  /*   ROS_INFO("[MrsStatus]: All params loaded!"); */
  /* } */


  // TIMERS
  status_timer_ = nh_.createTimer(ros::Rate(10), &MrsStatus::statusTimer, this);

  // SUBSCRIBERS
  state_subscriber_ = nh_.subscribe("state_in", 1, &MrsStatus::UavStateCallback, this, ros::TransportHints().tcpNoDelay());


  string                                                            topic_name = "/uav1/odometry/odom_main";
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
  callback           = [this, topic_name](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { GenericCallback(msg, topic_name); };
  generic_subscriber = nh_.subscribe(topic_name, 10, callback);

  initialized_ = true;
  ROS_INFO("[Mrs Status]: Node initialized!");

  refresh();
}

//}

/* statusTimer //{ */

void MrsStatus::statusTimer([[maybe_unused]] const ros::TimerEvent& event) {

  /* box(uav_state_win, '|', '-'); */

  /* mvwaddstr(uav_state_win, 2, 2, "hello"); */
  /* mvwaddstr(stdscr, 3, 3, to_string(counter).c_str()); */
  /* mvwaddstr(uav_state_win, 3, 3, to_string(counter).c_str()); */

  box(uav_state_win, '|', '-');
  uav_state_.pose.position.x = (rand() % 20000) + 0.23;
  /* if (fabs(uav_state_.pose.position.x) < 1000) { */
  /*   mvwprintw(uav_state_win, 1, 1, "X %6.2f", uav_state_.pose.position.x); */
  /* } else { */
  /*   mvwprintw(uav_state_win, 1, 1, "X %6.0e", uav_state_.pose.position.x); */
  /* } */

  PrintLimitedFloat(uav_state_win, 1, 1, "X %6.2f", uav_state_.pose.position.x);
  mvwprintw(uav_state_win, 2, 1, "Y %6.2f", uav_state_.pose.position.y);
  mvwprintw(uav_state_win, 3, 1, "Z %6.2f", uav_state_.pose.position.z);

  wrefresh(uav_state_win);
  refresh();
}

//}


/* PrintLimitedFloat() //{ */

void MrsStatus::PrintLimitedFloat(WINDOW* win, int y, int x, const char* format, ...) {
  va_list arg;
  mvwprintw(win, y, x, format, arg);
}

//}

/* UavStateCallback() //{ */

void MrsStatus::UavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {
  uav_state_counter_++;
  uav_state_ = *msg;
}

//}

/* GenericCallback() //{ */

void MrsStatus::GenericCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name) {
  counter++;
}

//}
//}

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

/* main() //{ */

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrs_status");

  /* std::vector<string> items; */
  /* items.push_back("1"); */
  /* items.push_back("2"); */
  /* items.push_back("3"); */
  /* items.push_back("4"); */

  initscr();
  cbreak();
  noecho();
  clear();

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

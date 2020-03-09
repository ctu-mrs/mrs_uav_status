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

#include <mrs_msgs/TrackerPoint.h>
#include <std_srvs/Trigger.h>
#include <mrs_lib/transformer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

using namespace std;

//}

/* class MrsStatus //{ */

class MrsStatus {

public:
  MrsStatus();

private:
  ros::NodeHandle nh_;

  ros::Timer blanket_timer_;

  void cmdOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void OdomCallback(const nav_msgs::OdometryConstPtr& msg);

  void blanketTimer(const ros::TimerEvent& event);

  ros::Subscriber cmd_odom_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;

  std::string _uav_name_;

  bool initialized_ = false;
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
  blanket_timer_ = nh_.createTimer(ros::Rate(10), &MrsStatus::blanketTimer, this);

  // SUBSCRIBERS
  cmd_odom_subscriber_ = nh_.subscribe("cmd_odom_in", 1, &MrsStatus::cmdOdomCallback, this, ros::TransportHints().tcpNoDelay());
  odom_subscriber_     = nh_.subscribe("odom_in", 1, &MrsStatus::OdomCallback, this, ros::TransportHints().tcpNoDelay());

  initialized_ = true;
  ROS_INFO("[Mrs Status]: Node initialized!");
}

//}

/* cmdOdomCallback() //{ */

void MrsStatus::cmdOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
}

//}

/* OdomCallback() //{ */

void MrsStatus::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
}

//}

/* blanketTimer //{ */

void MrsStatus::blanketTimer([[maybe_unused]] const ros::TimerEvent& event) {
}

//}

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

/* main() //{ */

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrs_status");

  std::vector<string> items;
  items.push_back("1");
  items.push_back("2");
  items.push_back("3");
  items.push_back("4");

  MrsStatus status;

  initscr();
  cbreak();
  noecho();
  clear();

  std::shared_ptr<WINDOW> menu_win(newwin(10, 12, 1, 1));

  Menu menu(menu_win, items);
  menu.Activate();
  endwin();

  /* char list[5][7] = {"One", "Two", "Three", "Four", "Five"}; */
  /* char item[7]; */
  /* int  ch, i = 0, width = 7; */


  /* menu_win = newwin(10, 12, 1, 1);  // create a new window */
  /* box(menu_win, '|', '-'); */

  /* for (i = 0; i < 5; i++) { */
  /*   if (i == 0) */
  /*     wattron(menu_win, A_STANDOUT);  // highlights the first item. */
  /*   else */
  /*     wattroff(menu_win, A_STANDOUT); */
  /*   sprintf(item, "%s", list[i]); */
  /*   mvwprintw(menu_win, i + 1, 2, "%s", item); */
  /* } */

  /* wrefresh(menu_win);  // update the terminal screen */

  /* keypad(menu_win, TRUE);  // enable keyboard input for the window. */
  /* curs_set(0);             // hide the default screen cursor. */

  /* while ((ch = wgetch(menu_win)) != 'q') { */

  /*   // right pad with spaces to make the items appear with even width. */
  /*   sprintf(item, "%s", list[i]); */
  /*   mvwprintw(menu_win, i + 1, 2, "%s", item); */
  /*   // use a variable to increment or decrement the value based on the input. */
  /*   if (ch == KEY_UP || ch == 'k') { */
  /*     i--; */
  /*     i = (i < 0) ? 4 : i; */
  /*   } else if (ch == KEY_DOWN || ch == 'j') { */
  /*     i++; */
  /*     i = (i > 4) ? 0 : i; */
  /*   } */
  /*   // now highlight the next item in the list. */
  /*   wattron(menu_win, A_STANDOUT); */

  /*   sprintf(item, "%s", list[i]); */
  /*   mvwprintw(menu_win, i + 1, 2, "%s", item); */
  /*   wattroff(menu_win, A_STANDOUT); */
  /* } */

  /* refresh(); */

  while (ros::ok()) {

    ros::spin();
    return 0;
  }
}

//

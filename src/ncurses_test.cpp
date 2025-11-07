/* INCLUDES //{ */

#include <ncurses.h>
#include <mrs_lib/timer_handler.h>

using namespace std;

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

/* class NcursesTest //{ */

class NcursesTest : public rclcpp::Node {

public:
  NcursesTest();

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::TimerBase::SharedPtr timer_init_;
  void                         timerInit();

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  void initialize();

  // | ------------------------- ncurses ------------------------ |

  WINDOW *createNewWin(int height, int width, int starty, int startx);

  void destroyWin(WINDOW *local_win);

  WINDOW *my_win_;
  int     start_x_;
  int     start_y_;
  int     height_;
  int     width_;

public:
  // | ------------------------- Timers ------------------------- |

  std::shared_ptr<TimerType> timer_main_;

  void timerMain();
};

//}

/* NcursesTest() //{ */

NcursesTest::NcursesTest() : Node("ncurses_test") {

  initscr();
  cbreak();
  timeout(1);
  keypad(stdscr, TRUE); /* I need that nifty F1 	*/
  noecho();

  timer_init_ = this->create_wall_timer(std::chrono::duration<double>(0.1s), std::bind(&NcursesTest::timerInit, this));
}

//}

/* createNewWin() //{ */

WINDOW *NcursesTest::createNewWin(int height, int width, int starty, int startx) {

  WINDOW *local_win;

  local_win = newwin(height, width, starty, startx);

  box(local_win, 0, 0); /* 0, 0 gives default characters
                         * for the vertical and horizontal
                         * lines			*/

  wrefresh(local_win); /* Show that box 		*/

  return local_win;
}

//}

/* destroyWin() //{ */

void NcursesTest::destroyWin(WINDOW *local_win) {

  /* box(local_win, ' ', ' '); : This won't produce the desired
   * result of erasing the window. It will leave it's four corners
   * and so an ugly remnant of window.
   */
  wborder(local_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
  /* The parameters taken are
   * 1. win: the window on which to operate
   * 2. ls: character to be used for the left side of the window
   * 3. rs: character to be used for the right side of the window
   * 4. ts: character to be used for the top side of the window
   * 5. bs: character to be used for the bottom side of the window
   * 6. tl: character to be used for the top left corner of the window
   * 7. tr: character to be used for the top right corner of the window
   * 8. bl: character to be used for the bottom left corner of the window
   * 9. br: character to be used for the bottom right corner of the window
   */
  wrefresh(local_win);
  delwin(local_win);
}

//}

/* timerInit() //{ */

void NcursesTest::timerInit() {

  node_  = this->shared_from_this();
  clock_ = node_->get_clock();

  cbkgrp_subs_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initialize();

  timer_init_->cancel();
}

//}

/* initialize() //{ */

void NcursesTest::initialize() {

  height_  = 3;
  width_   = 10;
  start_y_ = (LINES - height_) / 2; /* Calculating for a center placement */
  start_x_ = (COLS - width_) / 2;   /* of the window		*/

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&NcursesTest::timerMain, this);

    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(20.0, clock_), callback_fcn);
  }

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* timerMain() //{ */

void NcursesTest::timerMain() {

  destroyWin(my_win_);

  char ch = getch();

  switch (ch) {

  case 'h': {

    start_x_--;

    break;
  }

  case 'l': {

    start_x_++;

    break;
  }

  case 'j': {

    start_y_++;

    break;
  }

  case 'k': {

    start_y_--;

    break;
  }

  default: {
    break;
  }
  }

  my_win_ = createNewWin(height_, width_, start_y_, start_x_);
}

//}

} // namespace mrs_uav_status

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<mrs_uav_status::NcursesTest>();

  rclcpp::spin(node);

  /* while (rclcpp::ok()) { */

  /*   rclcpp::spin_some(node); */

  /*   // Small sleep to reduce CPU usage */
  /*   std::this_thread::sleep_for(std::chrono::milliseconds(1)); */
  /* } */

  rclcpp::shutdown();

  return 0;
}

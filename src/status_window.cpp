#include <status_window.h>

/* StatusWindow() //{ */

StatusWindow::StatusWindow(int lines, int cols, int begin_y, int begin_x, double rate_filter_coef, double desired_rate) {

  win_              = newwin(lines, cols, begin_y, begin_x);
  rate_filter_coef_ = rate_filter_coef;
  desired_rate_     = desired_rate;
}

//}

/* Redraw() //{ */

void StatusWindow::Redraw(void (MrsStatus::*fp)(WINDOW* win, double rate, short color), int &counter_in, MrsStatus* obj) {

  wclear(win_);

  rate_ -= rate_ / rate_filter_coef_;
  rate_ += (counter_in / ((ros::Time::now() - last_time_).toSec())) / rate_filter_coef_;

  if (!isfinite(rate_)) {
    rate_ = 0;
  }

  last_time_ = ros::Time::now();
  counter_in   = 0;

  wattron(win_, A_BOLD);

  box(win_, '|', '-');

  int tmp_color = RED;
  if (rate_ > 0.95 * desired_rate_) {
    tmp_color = GREEN;
  } else if (rate_ > 0.5 * desired_rate_) {
    tmp_color = YELLOW;
  }

  wattron(win_, COLOR_PAIR(tmp_color));

  if (rate_ < 0.01 * desired_rate_) {
    wattron(win_, A_BLINK);
    mvwprintw(win_, 0, 1, "!NO DATA!");
    wattroff(win_, A_BLINK);
  }

  (obj->*fp)(win_, rate_, tmp_color);

  wattroff(win_, COLOR_PAIR(tmp_color));
  wattroff(win_, A_BOLD);

  wrefresh(win_);
}

//}int(AB::*fp)(int,int)


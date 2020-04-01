#include <status_window.h>

/* Redraw() //{ */

void StatusWindow::Redraw(void (MrsStatus::*fp)(WINDOW* win, double rate, short color, int topic), MrsStatus* obj) {

  wclear(win_);

  double interval = (ros::Time::now() - last_time_).toSec();
  last_time_      = ros::Time::now();

  wattron(win_, A_BOLD);

  box(win_, 0, 0);

  int tmp_color = RED;
  
  if (topics_.size() == 0) {
    (obj->*fp)(win_, 0, NORMAL, 0);
  }

  for (unsigned long i = 0; i < topics_.size(); i++) {

    topics_[i].interval += interval;

    /* if (false) { */
    if (topics_[i].interval >= 1 / topics_[i].desired_rate) {

      double alpha           = 0.1;
      double rate_difference = (1 - (topics_[i].rate / topics_[i].desired_rate));

      if (rate_difference > 0.75) {
        alpha = 0.5;
      } else if (rate_difference > 0.1) {
        alpha = 0.25;
      }

      double new_rate = topics_[i].counter / (topics_[i].interval);

      (topics_[i].counter == 0) ? (topics_[i].zero_counter++) : (topics_[i].zero_counter = 0);

      topics_[i].interval = 0.0;
      topics_[i].counter = 0;

      if (topics_[i].zero_counter >= 2) {
        topics_[i].rate = 0.0;
      } else {
        topics_[i].rate = (alpha * new_rate) + (1.0 - alpha) * topics_[i].rate;
      }
    }

    tmp_color = RED;
    if (topics_[i].rate > 0.9 * topics_[i].desired_rate) {
      tmp_color = GREEN;
    } else if (topics_[i].rate > 0.5 * topics_[i].desired_rate) {
      tmp_color = YELLOW;
    }

    wattron(win_, COLOR_PAIR(tmp_color));

    if (!isfinite(topics_[i].rate) || topics_[i].rate < 0.05 || topics_[i].rate > 10000) {
      topics_[i].rate = 0;
    }
    (obj->*fp)(win_, topics_[i].rate, tmp_color, i);
  }

  wattroff(win_, COLOR_PAIR(tmp_color));
  wattroff(win_, A_BOLD);

  wrefresh(win_);
}

//}


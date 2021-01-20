#include <status_window.h>

/* Redraw() //{ */

void StatusWindow::Redraw(void (Status::*fp)(WINDOW* win, double rate, short color, int topic), bool light, Status * obj) {

  ros::Time time_now = ros::Time::now();
  double interval = (time_now - last_time_).toSec();
  last_time_ = time_now;

  werase(win_);


  wattron(win_, A_BOLD);

  wattroff(win_, A_STANDOUT);

  box(win_, 0, 0);

  if (light) {
    wattron(win_, A_STANDOUT);
  }
  int tmp_color = RED;

  if (topics_.size() == 0) {
    (obj->*fp)(win_, 0, NORMAL, 0);
  }

  for (unsigned long i = 0; i < topics_.size(); i++) {

    double avg_rate    = 0.0;
    avg_rate           = topics_[i].counter / interval;
    topics_[i].counter = 0;

    topics_[i].rates[topics_[i].rates_iterator] = avg_rate;
    topics_[i].rates_iterator++;

    if (topics_[i].rates_iterator >= topics_[i].rates.size()) {
      topics_[i].rates_iterator = 0;
    }

    avg_rate = 0;

    for (unsigned long j = 0; j < topics_[i].rates.size(); j++) {
      avg_rate += topics_[i].rates[j];
    }
    avg_rate = avg_rate / topics_[i].rates.size();

    tmp_color = RED;
    if (avg_rate > 0.9 * topics_[i].desired_rate) {
      tmp_color = GREEN;
    } else if (avg_rate > 0.5 * topics_[i].desired_rate) {
      tmp_color = YELLOW;
    }

    wattron(win_, COLOR_PAIR(tmp_color));

    (obj->*fp)(win_, avg_rate, tmp_color, i);
  }

  wattroff(win_, COLOR_PAIR(tmp_color));
  wattroff(win_, A_BOLD);

  wrefresh(win_);
}

//}

/* SetAttribute() //{ */

void StatusWindow::SetAttribute(int attrs) {
  wattron(win_, attrs);
}

//}

/* getWindow //{ */

WINDOW* StatusWindow::getWindow() {
  return win_;
}

//}

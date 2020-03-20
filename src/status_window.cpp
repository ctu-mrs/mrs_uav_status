#include <status_window.h>

/* StatusWindow() //{ */

StatusWindow::StatusWindow(int lines, int cols, int begin_y, int begin_x, std::vector<topic_rate> topic_rates) {

  win_         = newwin(lines, cols, begin_y, begin_x);
  topic_rates_ = topic_rates;
}

//}

/* Redraw() //{ */

void StatusWindow::Redraw(void (MrsStatus::*fp)(WINDOW* win, double rate, short color, int topic), MrsStatus* obj) {

  wclear(win_);

  double interval = (ros::Time::now() - last_time_).toSec();
  last_time_      = ros::Time::now();

  wattron(win_, A_BOLD);

  box(win_, '|', '-');

  int tmp_color = RED;

  for (unsigned long i = 0; i < topic_rates_.size(); i++) {

    topic_rates_[i].interval += interval;

    /* if (false) { */
    if (topic_rates_[i].interval >= 1 / topic_rates_[i].desired_rate) {

      double alpha           = 0.1;
      double rate_difference = (1 - (topic_rates_[i].rate / topic_rates_[i].desired_rate));

      if (rate_difference > 0.75) {
        alpha = 0.5;
      } else if (rate_difference > 0.1) {
        alpha = 0.25;
      }

      double new_rate = *topic_rates_[i].counter / (topic_rates_[i].interval);

      (*topic_rates_[i].counter == 0) ? (topic_rates_[i].zero_counter++) : (topic_rates_[i].zero_counter = 0);

      topic_rates_[i].interval = 0.0;
      *topic_rates_[i].counter = 0;

      if (topic_rates_[i].zero_counter >= 2) {
        topic_rates_[i].rate = 0.0;
      } else {
        topic_rates_[i].rate = (alpha * new_rate) + (1.0 - alpha) * topic_rates_[i].rate;
      }
    }

    tmp_color = RED;
    if (topic_rates_[i].rate > 0.9 * topic_rates_[i].desired_rate) {
      tmp_color = GREEN;
    } else if (topic_rates_[i].rate > 0.5 * topic_rates_[i].desired_rate) {
      tmp_color = YELLOW;
    }

    wattron(win_, COLOR_PAIR(tmp_color));

    if (!isfinite(topic_rates_[i].rate) || topic_rates_[i].rate < 0.05 || topic_rates_[i].rate > 10000) {
      topic_rates_[i].rate = 0;
    }
    (obj->*fp)(win_, topic_rates_[i].rate, tmp_color, i);
  }

  wattroff(win_, COLOR_PAIR(tmp_color));
  wattroff(win_, A_BOLD);

  wrefresh(win_);
}

//}


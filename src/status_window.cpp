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

    tmp_color = RED;
    if (topic_rates_[i].rate > 0.9 * topic_rates_[i].desired_rate) {
      tmp_color = GREEN;
    } else if (topic_rates_[i].rate > 0.5 * topic_rates_[i].desired_rate) {
      tmp_color = YELLOW;
    }

    wattron(win_, COLOR_PAIR(tmp_color));

    if (topic_rates_[i].rate < 0.01 * topic_rates_[i].desired_rate) {
      wattron(win_, A_BLINK);
      mvwprintw(win_, 0, 1, "!NO DATA!");
      wattroff(win_, A_BLINK);
    }


    topic_rates_[i].rate -= topic_rates_[i].rate /  (topic_rates_[i].desired_rate / 20);
    topic_rates_[i].rate += (*topic_rates_[i].counter / (interval)) /  (topic_rates_[i].desired_rate / 20);

    if (!isfinite(topic_rates_[i].rate) || topic_rates_[i].rate < 0.05 || topic_rates_[i].rate > 10000) {
      topic_rates_[i].rate = 0;
    }
    (obj->*fp)(win_, topic_rates_[i].rate, tmp_color, i);

    *topic_rates_[i].counter = 0;
  }

  wattroff(win_, COLOR_PAIR(tmp_color));
  wattroff(win_, A_BOLD);

  wrefresh(win_);
}

//}int(AB::*fp)(int,int)


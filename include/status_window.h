
#include <commons.h>

class Status;  // forward declaration

class StatusWindow {

public:
  StatusWindow(int lines, int cols, int begin_y, int begin_x, std::vector<topic>& topics, double window_rate) : topics_(topics) {
    win_         = newwin(lines, cols, begin_y, begin_x);
    window_rate_ = window_rate;
  };

  void    Redraw(void (Status::*fp)(WINDOW* win, double rate, short color, int topic), bool light, Status* obj);
  void    SetAttribute(int attrs);
  WINDOW* getWindow();

private:
  WINDOW*             win_;
  double              window_rate_;
  std::vector<topic>& topics_;
  ros::Time           last_time_ = ros::Time::now();
};


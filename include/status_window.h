
#include <commons.h>

class MrsStatus; //forward declaration

class StatusWindow {

public:
  StatusWindow(int lines, int cols, int begin_y, int begin_x, double rate_filter_coef, double desired_rate);

  void Redraw(void (MrsStatus::*fp)(WINDOW* win, double rate, short color, int topic), std::vector<std::reference_wrapper<int>> counters, MrsStatus* obj);

private:

  WINDOW* win_;

  double    rate_;
  double    rate_filter_coef_;
  double    desired_rate_;
  ros::Time last_time_ = ros::Time::now();
};


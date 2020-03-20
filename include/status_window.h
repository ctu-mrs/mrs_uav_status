
#include <commons.h>

class MrsStatus;  // forward declaration

class StatusWindow {

public:
  StatusWindow(int lines, int cols, int begin_y, int begin_x, std::vector<topic_rate> topic_rates);

  void Redraw(void (MrsStatus::*fp)(WINDOW* win, double rate, short color, int topic), MrsStatus* obj);

private:
  WINDOW* win_;

  std::vector<topic_rate>                  topic_rates_;
  ros::Time                                last_time_ = ros::Time::now();
};


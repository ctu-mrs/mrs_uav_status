
#include <commons.h>

class MrsStatus;  // forward declaration

class StatusWindow {

public:
  StatusWindow(int lines, int cols, int begin_y, int begin_x, std::vector<topic> topics);

  void Redraw(void (MrsStatus::*fp)(WINDOW* win, double rate, short color, int topic), MrsStatus* obj);

private:
  WINDOW* win_;

  std::vector<topic>                  topics_;
  ros::Time                                last_time_ = ros::Time::now();
};


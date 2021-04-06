#include <commons.h>

class TopicInfo {

public:
  TopicInfo(double window_rate_in, int buffer_len);
  double GetHz();
  void Count();

private:

  ros::Time           last_time_;
  int                 counter_;
  double              window_rate_;
  std::vector<double> rates_;
  size_t              rates_iterator_;
};

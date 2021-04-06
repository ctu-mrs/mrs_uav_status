#include <topic_info.h>

/* Menu() //{ */

TopicInfo::TopicInfo(double window_rate_in, int buffer_len) {
  window_rate_ = window_rate_in;
  rates_.resize(buffer_len * int(window_rate_));
  rates_.assign(rates_.size(), 0.0);
  rates_iterator_ = 0;
  last_time_      = ros::Time::now();
  counter_        = 0;
}

double TopicInfo::GetHz() {
  ros::Time time_now   = ros::Time::now();
  double    interval   = (time_now - last_time_).toSec();
  last_time_ = time_now;

  double avg_rate    = counter_ / interval;
  counter_ = 0;

  rates_[rates_iterator_] = avg_rate;
  rates_iterator_++;

  if (rates_iterator_ >= rates_.size()) {
    rates_iterator_ = 0;
  }

  avg_rate = 0.0;

  for (unsigned long i = 0; i < rates_.size(); i++) {
    avg_rate += rates_[i];
  }
  avg_rate = avg_rate / double(rates_.size());
  return avg_rate;
}

void TopicInfo::Count() {
  counter_++;
}
//}

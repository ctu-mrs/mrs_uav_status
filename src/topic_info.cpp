#include <commons.h>


/* TopicInfo() //{ */

TopicInfo::TopicInfo(double window_rate_in, int buffer_len, double desired_rate_in) {
  window_rate_  = window_rate_in;
  desired_rate_ = desired_rate_in;
  rates_.resize(buffer_len * int(window_rate_));
  rates_.assign(rates_.size(), 0.0);
  rates_iterator_     = 0;
  last_time_          = ros::Time::now();
  counter_            = 0;
  topic_name_         = "NOT_DEFINED";
  topic_display_name_ = "NOT_DEFINED";
}

//}

/* TopicInfo() //{ */

TopicInfo::TopicInfo(double window_rate_in, int buffer_len, double desired_rate_in, std::string topic_name_in, std::string topic_display_name_in) {
  window_rate_  = window_rate_in;
  desired_rate_ = desired_rate_in;
  rates_.resize(buffer_len * int(window_rate_));
  rates_.assign(rates_.size(), 0.0);
  rates_iterator_     = 0;
  last_time_          = ros::Time::now();
  counter_            = 0;
  topic_name_         = topic_name_in;
  topic_display_name_ = topic_display_name_in;
}

//}

/* GetHz //{ */

std::tuple<double, int16_t> TopicInfo::GetHz() {
  ros::Time time_now = ros::Time::now();
  double    interval = (time_now - last_time_).toSec();

  if (interval == 0.0) {
    return std::make_tuple(0.0, RED);
  }

  last_time_ = time_now;

  double avg_rate = counter_ / interval;
  counter_        = 0;

  rates_[rates_iterator_] = avg_rate;
  rates_iterator_++;

  if (rates_iterator_ >= rates_.size()) {
    rates_iterator_ = 0;
  }

  avg_rate = 0.0;

  for (unsigned long i = 0; i < rates_.size(); i++) {
    avg_rate += rates_[i];
  }

  if (rates_.size() == 0) {
    avg_rate = 0.0;
  } else {
    avg_rate = avg_rate / double(rates_.size());
  }

  int16_t color = RED;

  if (avg_rate > 0.9 * desired_rate_) {
    color = GREEN;
  } else if (avg_rate > 0.5 * desired_rate_) {
    color = YELLOW;
  }
  return std::make_tuple(avg_rate, color);
}

//}

/* GetTopicName //{ */

std::string TopicInfo::GetTopicName() {
  return topic_name_;
}

//}

/* GetTopicName //{ */

std::string TopicInfo::GetTopicDisplayName() {
  return topic_display_name_;
}

//}

/* Count //{ */

void TopicInfo::Count() {
  counter_++;
}

//}


#include <ncurses.h>

#include <ros/ros.h>
#include <ros/package.h>

#define NORMAL 100
#define GREEN 101
#define RED 102
#define YELLOW 103
#define BLUE 104

#define BACKGROUND_DEFAULT -1
#define BACKGROUND_TRUE_BLACK 16

#define COLOR_NICE_RED 196
#define COLOR_NICE_GREEN 82
#define COLOR_NICE_BLUE 33
#define COLOR_NICE_YELLOW 220

struct topic
{
  std::string          topic_name;
  std::shared_ptr<int> counter;
  double               rate;
  double               interval;
  double               desired_rate;
  int                  zero_counter;

  topic(std::shared_ptr<int> counter_in, double desired_rate_in) {
    topic_name   = "NOT DEFINED";
    counter      = counter_in;
    desired_rate = desired_rate_in;
    rate         = 0.0;
    interval     = 0.0;
    zero_counter = 0;
  }

  topic(std::string topic_name_in, std::shared_ptr<int> counter_in, double desired_rate_in) {
    topic_name   = topic_name_in;
    counter      = counter_in;
    desired_rate = desired_rate_in;
    rate         = 0.0;
    interval     = 0.0;
    zero_counter = 0;
  }
};

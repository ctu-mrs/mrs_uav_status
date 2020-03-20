
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

struct topic_rate {
  std::shared_ptr<int> counter;
  double rate;
  double desired_rate;
}; 

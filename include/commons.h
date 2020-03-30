#ifndef COMMONS_H
#define COMMONS_H

#define KEY_ENT 10
#define KEY_ESC 27

#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>

#include <tuple>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

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
  std::string topic_name;
  std::string topic_display_name;
  int         counter;
  double      rate;
  double      interval;
  double      desired_rate;
  int         zero_counter;

  topic(double desired_rate_in) {
    topic_name         = "NOT DEFINED";
    topic_display_name = "NOT DEFINED";
    desired_rate       = desired_rate_in;
    counter            = 0;
    rate               = 0.0;
    interval           = 0.0;
    zero_counter       = 0;
  }

  topic(std::string topic_name_in, std::string topic_display_name_in, double desired_rate_in) {
    topic_name         = topic_name_in;
    topic_display_name = topic_display_name_in;
    desired_rate       = desired_rate_in;
    counter            = 0;
    rate               = 0.0;
    interval           = 0.0;
    zero_counter       = 0;
  }
};

struct service
{
  std::string        service_name;
  std::string        service_display_name;
  ros::ServiceClient service_client;

  service(std::string name_in, std::string display_name_in) {
    service_name         = name_in;
    service_display_name = display_name_in;
  }
};

#endif

#pragma once
#ifndef COMMONS_H
#define COMMONS_H

#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include <form.h>

#include <tuple>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#define KEY_ENT 10
#define KEY_ESC 27
/* #define KEY_BACKSPACE 263 */
#define KEY_DELETE 330

#define NORMAL 100
#define FIELD 101
#define GREEN 102
#define RED 103
#define YELLOW 104
#define BLUE 105

#define BACKGROUND_DEFAULT -1
#define BACKGROUND_TRUE_BLACK 16

#define COLOR_NICE_RED 196

#define COLOR_NICE_GREEN 82
#define COLOR_DARK_GREEN 2

#define COLOR_NICE_BLUE 33
#define COLOR_DARK_BLUE 21

#define COLOR_NICE_YELLOW 220
#define COLOR_DARK_YELLOW 172

#define BUFFER_SECS_LEN 4

struct topic
{
  std::string topic_name;
  std::string topic_display_name;
  int         counter;
  double      desired_rate;
  int         zero_counter;

  unsigned long       rates_iterator;
  std::vector<double> rates;

  ros::Time last_time_;

  topic(double desired_rate_in, double window_rate) {
    topic_name         = "NOT DEFINED";
    topic_display_name = "NOT DEFINED";
    desired_rate       = desired_rate_in;
    counter            = 0;
    zero_counter       = 0;
    rates_iterator     = 0;
    rates.resize(BUFFER_SECS_LEN * window_rate);
    last_time_ = ros::Time::now();
  }

  topic(std::string topic_name_in, std::string topic_display_name_in, double desired_rate_in, double window_rate) {
    topic_name         = topic_name_in;
    topic_display_name = topic_display_name_in;
    desired_rate       = desired_rate_in;
    counter            = 0;
    zero_counter       = 0;
    rates_iterator     = 0;
    rates.resize(BUFFER_SECS_LEN * window_rate);
    last_time_ = ros::Time::now();
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

/* struct string_info */
/* { */
/*   std::string publisher_name; */
/*   std::string display_string; */
/*   ros::Time   last_time; */

/*   string_info(std::string publisher_name_in, std::string display_string_in) { */
/*     publisher_name = publisher_name_in; */
/*     display_string = display_string_in; */
/*     last_time      = ros::Time::now(); */
/*   } */
/* }; */

#endif

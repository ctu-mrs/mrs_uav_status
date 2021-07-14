#pragma once
#ifndef COMMONS_H
#define COMMONS_H

#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include <form.h>

#include <utility>
#include <tuple>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <thread>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/UavStatusShort.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/OdometryDiag.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/CustomTopic.h>

#include <std_msgs/String.h>

#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>

#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>

#include <tf2_msgs/TFMessage.h>
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
#define ALWAYS_RED 106

#define BACKGROUND_DEFAULT -1
#define BACKGROUND_TRUE_BLACK 16

#define COLOR_NICE_RED 196
#define COLOR_DARK_RED 88

#define COLOR_NICE_GREEN 82
#define COLOR_DARK_GREEN 2

#define COLOR_NICE_BLUE 33
#define COLOR_DARK_BLUE 21

#define COLOR_NICE_YELLOW 220
#define COLOR_DARK_YELLOW 172

#define BUFFER_SECS_LEN 4

class TopicInfo {

public:
  TopicInfo(double window_rate_in, int buffer_len, double desired_rate_in);
  TopicInfo(double window_rate_in, int buffer_len, double desired_rate_in, std::string topic_name_in, std::string topic_display_name_in);
  std::string                 GetTopicName();
  std::string                 GetTopicDisplayName();
  std::tuple<double, int16_t> GetHz();
  void                        Count();

private:
  ros::Time           last_time_;
  int                 counter_;
  std::string         topic_name_;
  std::string         topic_display_name_;
  double              window_rate_;
  double              desired_rate_;
  std::vector<double> rates_;
  size_t              rates_iterator_;
};

struct service
{
  std::string service_name;
  std::string service_display_name;

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> service_client;

  service(std::string name_in, std::string display_name_in) {
    service_name         = name_in;
    service_display_name = display_name_in;
  }
};

struct topic_status
{
  ros::Time           last_time;
  int                 counter;
  double              window_rate;
  std::vector<double> rates;
  size_t              rates_iterator = 0;

  topic_status(double window_rate_in, int buffer_len) {
    window_rate = window_rate_in;
    rates.resize(buffer_len * int(window_rate));
    rates.assign(rates.size(), 0.0);
    rates_iterator = 0;
    last_time      = ros::Time::now();
    counter        = 0;
  }
};

struct string_info
{
  std::string publisher_name;
  std::string id;
  std::string display_string;
  bool        persistent;
  ros::Time   last_time;

  string_info(std::string publisher_name_in, std::string display_string_in, std::string id_in, bool persistent_in) {
    publisher_name = publisher_name_in;
    display_string = display_string_in;
    id             = id_in;
    persistent     = persistent_in;
    last_time      = ros::Time::now();
  }
};

#endif

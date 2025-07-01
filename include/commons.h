#pragma once
#ifndef COMMONS_H
#define COMMONS_H

#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include <form.h>

/* #include <utility> */
#include <tuple>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>

/* #include <iostream> */
/* #include <fstream> */
/* #include <thread> */

#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>

#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/custom_topic.hpp>

#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>

#include <tf2_msgs/msg/tf_message.hpp>

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
#define COLOR_DARK_BLUE 19

#define COLOR_NICE_YELLOW 220
#define COLOR_DARK_YELLOW 172

#define BUFFER_SECS_LEN 4

class TopicInfo {

public:
  TopicInfo();
  TopicInfo(rclcpp::Node::SharedPtr node, double window_rate_in, int buffer_len, double desired_rate_in);
  TopicInfo(rclcpp::Node::SharedPtr node, double window_rate_in, int buffer_len, double desired_rate_in, std::string topic_name_in, std::string topic_display_name_in);

  std::string                 GetTopicName();
  std::string                 GetTopicDisplayName();
  std::tuple<double, int16_t> GetHz();
  void                        Count();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time            last_time_;
  int                     counter_;
  std::string             topic_name_;
  std::string             topic_display_name_;
  double                  window_rate_;
  double                  desired_rate_;
  std::vector<double>     rates_;
  size_t                  rates_iterator_;
};

struct service
{
  std::string service_name;
  std::string service_display_name;

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> service_client;

  service(std::string name_in, std::string display_name_in) {
    service_name         = name_in;
    service_display_name = display_name_in;
  }
};

struct topic_status
{
  rclcpp::Time            last_time;
  int                     counter;
  double                  window_rate;
  std::vector<double>     rates;
  size_t                  rates_iterator = 0;
  rclcpp::Node::SharedPtr node;

  topic_status(double window_rate_in, int buffer_len) {
    window_rate = window_rate_in;
    rates.resize(buffer_len * int(window_rate));
    rates.assign(rates.size(), 0.0);
    rates_iterator = 0;
    last_time      = node->get_clock()->now();
    counter        = 0;
  }
};

struct string_info
{
  std::string  publisher_name;
  std::string  id;
  std::string  display_string;
  bool         persistent;
  rclcpp::Time last_time;

  string_info(rclcpp::Time last_time, std::string publisher_name_in, std::string display_string_in, std::string id_in, bool persistent_in) {
    publisher_name  = publisher_name_in;
    display_string  = display_string_in;
    id              = id_in;
    persistent      = persistent_in;
    this->last_time = last_time;
  }
};

struct node_info
{
  std::string node_name;
  int         node_pid;
  float       node_cpu_usage;
  long        last_utime;
  long        last_stime;

  node_info(std::string node_name_in) {
    node_name      = node_name_in;
    node_pid       = 0;
    node_cpu_usage = 0.0;
    last_utime     = 0;
    last_stime     = 0;
  }
};


#endif

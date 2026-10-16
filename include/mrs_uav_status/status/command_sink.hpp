#pragma once

/* includes //{ */

#include <functional>
#include <string>
#include <vector>

//}

namespace mrs_uav_status::status
{

// Outbound ROS actions UavStatusCore can trigger, injected by ros_status so status/ never links
// against rclcpp service/message types. Every call is synchronous.
struct CommandSink
{
  struct ServiceResult
  {
    bool        success = false;
    std::string message;
  };

  // One config-driven Trigger-backed menu entry (from mrs_uav_status/menu/service_list).
  struct NamedService
  {
    std::string                    display_name;
    std::function<ServiceResult()> call;
  };

  std::function<ServiceResult(double x, double y, double z, double heading, const std::string &frame_id)> sendGoto;
  // Fire-and-forget, matches today's behavior (the response is not checked at the call site).
  std::function<void(double vx, double vy, double vz, double heading_rate, const std::string &frame_id)> sendVelocityReference;
  std::function<ServiceResult(const std::string &value)>                                                 setConstraints;
  std::function<ServiceResult(const std::string &value)>                                                 setGains;
  std::function<ServiceResult(const std::string &value)>                                                 setController;
  std::function<ServiceResult(const std::string &value)>                                                 setTracker;
  std::function<ServiceResult(const std::string &value)>                                                 setEstimator;
  std::function<ServiceResult()>                                                                         hover;

  std::vector<NamedService> extra_services;
};

} // namespace mrs_uav_status::status

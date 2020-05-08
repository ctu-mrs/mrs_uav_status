# MRS UAV Status [![Build Status](https://travis-ci.com/ctu-mrs/mrs_uav_status.svg?branch=master)](https://travis-ci.com/ctu-mrs/mrs_uav_status)

![](.fig/thumbnail.jpg)

## Real-time Terminal User Interface for monitoring and control

* runs on the UAV within its TMUX session
* usable through SSH
* written in C++ with curses for windows
* reports useful data and states of the UAV
* monitors ROS topic rates
  * predefine in a config file
  * dynamically set during run-time by a service
* can show arbitrary data from any node using a provided topic
* allows calling services
  * predefined for references, landing, takeoff, etc.
  * dynamically set during run-time by a service
* allows controlling the UAV using WSAD-like control scheme (press <shift>R to activate)
* vim-like key bindings

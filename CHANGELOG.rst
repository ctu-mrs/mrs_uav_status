^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_status
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-08-13)
------------------
* Refactored into layers: UavStatus (all ROS I/O) drives UavStatusCore (the state machine and every message snapshot), which drives NcursesTui (pure rendering) through an abstract TuiActions interface
* Added an automated test suite (test_uav_status_core) and a build-time check enforcing that tui/ and status/ stay free of ROS types
* Fixed numerous fields (armed/mode, mass estimate, thrust, battery, GNSS/magnetometer, HW API rate, control manager, ROS Node CPU/Sensors panes, position/estimator display) rendering stale or default data as if real instead of showing "NO DATA", by gating each on its own topic's freshness instead of a shared/aliased flag
* Fixed crash risks from malformed service_list config entries and a corrupted display-config file
* Fixed several display glitches: overlapping/garbled disk-space text, printNoData() clobbering the caller's color, the ToF NO-DATA fallback overlapping the UAV name field, "Mode NO DATA" printed on the wrong row, and NaN/"unknown" shown instead of NO DATA/ERR
* Fixed submenu CANCEL backing out of the whole menu instead of just one level
* Fixed display_string entries only being pruned while the GNSS pane was selected, instead of every tick
* Sensors pane now shows each sensor's message and sorts by severity; remote mode auto-shows its control guide instead of the generic help hint
* Contributors: Filip Stojanovic, Tomas Baca, Viktor Walter, Vojtech Spurny

2.0.0 (2026-07-13)
------------------
* Ported to ROS2 (ament_cmake, rclcpp, mrs_lib::Node)
* Assorted fixes and cleanup following the initial port
* Contributors: Dan Hert, Filip Stojanovic, Marlon Rivera, Matej Petrlik, pum1k, Tomas Baca, Vit Kratky, Vojtech Spurny

1.0.4 (2023-01-20)
------------------
* updated ci, updated readme
* bugfix
* display improvements
* fixed launch, env->optenv
* small update
* hiding the config file
* added persistent tmux displays
* updated tmux display mode
* adding tmux throughput mode
* added safety area check
* added collision avoidance display
* finishing minimalistic mode
* updates
* minimialistic mode
* Contributors: Dan Hert, Tomas Baca

1.0.3 (2022-05-09)
------------------
* updated dependencies
* updated transformer interface
* refactored agains the new transformer
* + install in cmakelists
* added gimbal mode
* fixed colorblind mode
* Contributors: Dan Hert, Tomas Baca

1.0.2 (2021-10-04)
------------------
* added cpu temperature
* add respawn=true to acquisition.launch
* updated process cpu load
* added rosnode shitlist
* fill time in uav_status msg
* fixed tf static problem, added more generic topics
* Add publishing of cpuload
* updated tracker and controller switching (human switchable)
* updated synchro tmux
* Added basic Control error display, and more battery stats
* fixed remote mode setting from odom issue
* fixing land home problem
* moved msg time indicator
* fixed display of controllers/trackers that are not in the list of available controllers/trackers
* added more params to custom string msgs
* added confirmation dialogs to land, land_home and other trig services
* added change odometry source service
* updated mass loading for simulation
* pass config_file arg in status.launch
* Contributors: Dan Hert, Daniel Hert, Matej Petrlik, Pavel Petracek, Tomas Baca, Vit Kratky

1.0.1 (2021-05-16)
------------------
* updated ros::shutdown
* added path to nimbro in tmux
* fix flicker and arrow keys
* added multilander capabilities
* added service client handler from mrs_lib
* split to two sections
* added global remote mode
* Contributors: Daniel Hert, Matouš Vrba, Tomas Baca, mrs drone

1.0.0 (2021-03-18)
------------------
* Major release
* allow loading of custom configs overriding only part of variables of the default.yaml file
* remove ALOAM from static TF list
* Contributors: Pavel Petracek

0.0.6 (2021-03-16)
------------------
* python -> c++ implementation
* + service call handling
* + topic visualization
* Contributors: Dan Hert, Matej Petrlik, Pavel Petracek, Robert Penicka, Tomas Baca, Viktor Walter, afzal

0.0.5 (2020-02-26)
------------------
* Blinkengripper
* Contributors: Dan Hert

0.0.4 (2020-02-18)
------------------
* added five sec timer
* 1hz
* Added data overload display
* flight timer fix
* fixed flight timer
* Added param window shortening
* Flight timer working
* updated gain and constraint info
* config param in launch as absolute path
* added thermals
* added odometry estimators
* added gripper
* add config_file parameter to launch file
* fixed disk space
* added vel and acc bars
* new frame_id in odometry
* updated uav mass readout
* Contributors: Dan Hert, Pavel Petracek, Tomas Baca, delta, uav64, uav66

0.0.3 (2019-10-25)
------------------
* added bumper stuff
* added uvdar
* small gps fix
* gps update
* added SENSORS variable config
* added collision avoidance
* updated tracker and controller status, fixed mass
* Contributors: Dan Hert, Tomas Baca, Viktor Walter, uav42, uav43, uav46, uav64

0.0.2 (2019-07-01)
------------------
* + battery level
* Add garmin up and rplidar for naki
* added battery
* Fix thrust glitch
* added respawns
* added rtk, yaw and hopefully thrust
* VIO launch and config
* Add config/launch for NAKI
* Contributors: Dan Hert, Daniel Heřt, Matej Petrlik, NAKI, Pavel Petracek, Tomas Baca, mrs, uav10, uav5

0.0.1 (2019-05-20)
------------------

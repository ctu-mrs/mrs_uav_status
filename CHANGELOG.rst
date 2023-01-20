^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_status
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

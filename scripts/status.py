#!/usr/bin/env python
# -*- coding: utf-8 -*- 

#TODO fix crashing when odom dies

import rospkg
import rospy
import curses
import os
import time
import multiprocessing as mp
import subprocess
import rosnode
import tf

from rostopic import ROSTopicHz

from mrs_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *

white = 0 
red = 0
green = 0
yellow = 0

# #{ get_cpu_load()

def get_cpu_load(cpu_load_queue, queue_lock):
    run = True
    while run:
        bashCommand = "vmstat 1 2"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        output = output.split()[-3]
        queue_lock.acquire()
        if cpu_load_queue.qsize() > 3:
            run = False
        else:
            cpu_load_queue.put(100-int(output))
        queue_lock.release()

cpu_load_queue = mp.Queue()
queue_lock = mp.Lock()
process = mp.Process(target=get_cpu_load, args=(cpu_load_queue, queue_lock))

# #} end of get_cpu_load()

class Status:

    # #{ Callbacks

    def MultiCallback(self, data, callback_id):
        self.count_list[callback_id] = self.count_list[callback_id] + 1

    def ConstraintsCallback(self, data):
        self.constraints = data
        self.count_constraints= self.count_constraints + 1

    def GainsCallback(self, data):
        self.gains = data
        self.count_gains = self.count_gains + 1

    def ControlManagerDiagnostics(self, data):
        self.controller_status = data.controller_status
        self.count_controller = self.count_controller + 1

        self.tracker_status = data.tracker_status
        self.count_tracker = self.count_tracker + 1

    def AttitudeCmdCallback(self, data):
        self.attitude_cmd = data
        self.count_attitude = self.count_attitude + 1

    def UavStateCallback(self, data):
        self.uav_state = data
        self.count_uav_state = self.count_uav_state + 1

    def StateCallback(self, data):
        self.mavros_state = data
        self.count_state = self.count_state + 1

    def BestposCallback(self, data):
        self.bestpos = data
        self.count_bestpos = self.count_bestpos + 1

    def BatteryCallback(self, data):
        self.battery = data
        self.count_battery = self.count_battery + 1

    def MPCstatusCallback(self, data):
        self.mpcstatus = data
        self.count_mpcstatus = self.count_mpcstatus + 1

    def gripperCallback(self, data):
        self.gripper = data
        self.count_gripper = self.count_gripper + 1

    def GPSCallback(self, data):
        self.gpsdata = data
        self.count_gpsdata = self.count_gpsdata + 1
        self.has_gps = True

    def uvdarCallback(self, data):
        self.uvdar = data
        self.count_uvdar = self.count_uvdar + 1

    def bumperCallback(self, data):
        self.bumper_status = data
        self.bumper_repulsing |= self.bumper_status.repulsing
        self.bumper_constraining |= self.bumper_status.modifying_reference

    def currConstCallback(self, data):
        self.curr_const = data
        self.count_curr_const = self.count_curr_const + 1

    def balloonCallback(self, data):
        self.balloon = data
        self.count_balloon = self.count_balloon + 1

    def controlErrorCallback(self, data):
        self.control_error = data
        self.count_control_error = self.count_control_error + 1

    # #} end of Callbacks

    # #{ ErrorShutdown

    def ErrorShutdown(self, message, stdscr, color):
        stdscr.clear()
        stdscr.attron(color)
        stdscr.attron(curses.A_BLINK)
        stdscr.attron(curses.A_BOLD)
        stdscr.addstr(1, 0,str(message))
        stdscr.refresh()
        time.sleep(10)
        rospy.signal_shutdown('Quit') 

    # #} end of ErrorShutdown

    # #{ status

    def status(self, stdscr):


        # Initialize some lists
        topic_list = []
        self.name_list = []
        self.hz_list = []
        self.hz_list_modifiers = []
        sub_list = []
        dark_mode = True
        colorblind_mode = True

        # Initialize curses
        curses.initscr()
        curses.start_color()
        curses.use_default_colors()

        for i in range(0, curses.COLORS):
            curses.init_pair(i + 1, i, -1)

        # Get parameters from config file and put them in lists


        self.MRS_STATUS = str(self.MRS_STATUS).replace(',', ' ') 
        status_list = str(self.MRS_STATUS).split(' ')
        window_list = []

        # #{ Sensor list

        self.param_list = rospy.get_param('~want_hz', "")
        self.SENSORS = str(self.SENSORS).replace(',', ' ') 
        self.sensor_list = str(self.SENSORS).split(' ')

        if str(self.PIXGARM) == "true":
            self.param_list.insert(0, "mavros/distance_sensor/garmin Garmin_pix 80+")

        if str(self.PIXGARM) == "false" and 'garmin_down' in self.sensor_list:
            self.param_list.insert(0, "garmin/range Garmin_Down 80+")
        
        if 'garmin_up' in self.sensor_list:
            self.param_list.insert(0, "garmin/range_up Garmin_Up 80+")

        if 'realsense_brick' in self.sensor_list:
            self.param_list.insert(0, "rs_d435/color/camera_info Realsense_Brick 25+")
        
        if 'realsense_front' in self.sensor_list:
            self.param_list.insert(0, "rs_d435/color/camera_info Realsense_Front 25+")
        
        if 'bluefox_brick' in self.sensor_list:
            self.param_list.insert(0, "bluefox_brick/camera_info Bluefox_Brick 25+")
        
        if 'bluefox_optflow' in self.sensor_list:
            self.param_list.insert(0, "bluefox_optflow/camera_info Bluefox_Optflow 60+")
            self.param_list.insert(0, "optic_flow/velocity Optic_flow 60+")
        
        if 'trinocular_thermal' in self.sensor_list:
            self.param_list.insert(0, "thermal/top/rgb_image Thermal_Top 15+")
            self.param_list.insert(0, "thermal/middle/rgb_image Thermal_Middle 15+")
            self.param_list.insert(0, "thermal/bottom/rgb_image Thermal_Bottom 15+")
        
        if 'rplidar' in self.sensor_list:
            self.param_list.insert(0, "rplidar/scan Rplidar 10+")
        
        if str(self.BLUEFOX_UV_LEFT) != "":
            self.param_list.insert(0, "uvdar_bluefox/left/camera_info Bluefox_UV_left 70+")
        
        if str(self.BLUEFOX_UV_RIGHT) != "":
            self.param_list.insert(0, "uvdar_bluefox/right/camera_info Bluefox_UV_right 70+")
        
        if str(self.ODOMETRY_TYPE) == "gps":
            self.param_list.insert(0, "mavros/global_position/global PX4 GPS 100")
        tmp_string = ""
        for i in self.param_list:
            topic_list.append(i.rsplit()[0])
            tmp = i.rsplit(' ', 1)[0]
            self.name_list.append(tmp.split(' ', 1)[1])
            tmp_string = i.rsplit()[-1]
            if tmp_string[-1] == "-" or tmp_string[-1] == "+":
                self.hz_list_modifiers.append(tmp_string[-1])
                tmp_string = tmp_string[:-1]
            else:
                self.hz_list_modifiers.append("0")
            self.hz_list.append(float(tmp_string))
        if len(self.name_list) > 7:
            self.many_param_flag = True
            for i in range(0, len(self.name_list)):
                self.name_list[i] = self.name_list[i].translate(None, 'aeiouAEIOU_- ')
                self.name_list[i] = self.name_list[i][0:6]
        
        # #} end of Sensor list

        # #{ Colorscheme
        
        profile_list = str(self.PROFILES_BOTH).split(' ')
        dark_mode = False
        if 'COLORSCHEME_DARK' in profile_list:
            dark_mode = True
        
        # #} end of Colorscheme

        # dark_mode = rospy.get_param('~dark_mode', True)
        colorblind_mode = rospy.get_param('~colorblind_mode', True)

        # stdscr.addstr(10, 40," " + str(tmp_string))

        # Create ROSTopicHz and subscribers defined by the loaded config
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/diagnostics", ControlManagerDiagnostics, self.ControlManagerDiagnostics)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/attitude_cmd", AttitudeCommand, self.AttitudeCmdCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/odometry/uav_state", UavState, self.UavStateCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/state", State, self.StateCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/gain_manager/diagnostics", GainManagerDiagnostics, self.GainsCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/constraint_manager/diagnostics", ConstraintManagerDiagnostics, self.ConstraintsCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/tersus/bestpos", Bestpos, self.BestposCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/battery", BatteryState, self.BatteryCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/global_position/global", NavSatFix, self.GPSCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/bumper_status", BumperStatus, self.bumperCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/current_constraints", TrackerConstraints, self.currConstCallback)
        
        if 'uvdar' in self.sensor_list:
            rospy.Subscriber("/" + str(self.UAV_NAME) + "/uvdar/targetSeenCount", Int16, self.uvdarCallback)
    
        for i in range(0, len(self.param_list)):
            if(str(topic_list[i][0]) == "/"):
                sub_list.append(rospy.Subscriber(str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            else:
                sub_list.append(rospy.Subscriber("/" + str(self.UAV_NAME) + "/" + str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            self.count_list.append(0)

        # #{ Default Windows

        #---------------ODOM WINDOW---------------#

        begin_x = 0; begin_y = 5
        height = 3; width = 42
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.odomWin, 5, begin_x + width + 1)
        window_list.append(tmp_tuple);

        #---------------CONTROL WINDOW---------------#

        begin_x = 0; begin_y = 1
        height = 3; width = 26
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.controlWin, 1, begin_x + width + 1)
        window_list.append(tmp_tuple);

        ##---------------NAMES WINDOW---------------#

        begin_x = 0; begin_y = 0
        height = 1; width = 30
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.namesWin, 1, begin_x + width + 1)
        window_list.append(tmp_tuple);
        
        ##---------------PIXHAWK WINDOW---------------#

        begin_x = 26; begin_y = 1
        height = 3; width = 18
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.pixhawkWin, 1, begin_x)
        window_list.append(tmp_tuple);
        
        ##---------------SENSOR WINDOW---------------#

        begin_x = 43; begin_y = 1
        height = 12; width = 34
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.winConfTopics, 1, begin_x)
        window_list.append(tmp_tuple);
        
        ##---------------CPU LOAD WINDOW---------------#

        begin_x = 76; begin_y = 1
        height = 1; width = 20
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.cpuLoadWin, 1, begin_x)
        window_list.append(tmp_tuple);
        

        ##---------------DISK WINDOW---------------#

        begin_x = 76; begin_y = 2
        height = 1; width = 20
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.winDisk, 1, begin_x)
        window_list.append(tmp_tuple);

        ##---------------GPS WINDOW---------------#

        begin_x = 76; begin_y = 4
        height = 2; width = 20
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.winGps, 1, begin_x)
        window_list.append(tmp_tuple);

        #---------------MASS WINDOW---------------#

        begin_x = 76; begin_y = 7
        height = 1; width = 20
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.massWin, 1, begin_x)
        window_list.append(tmp_tuple);

        ##---------------TIME WINDOW---------------#

        begin_x = 26; begin_y = 0
        height = 1; width = 40
        tmp_win = curses.newwin(height, width, begin_y, begin_x)
        tmp_tuple = (tmp_win, self.timeWin, 1, begin_x + width + 1)
        window_list.append(tmp_tuple);
        
        
        # #} end of Default Windows

        begin_x = 95; begin_y = 0
        width = 80

        if 'readme' in status_list:
            height = 4
            tmp_win = curses.newwin(height, width, begin_y, begin_x)
            tmp_tuple = (tmp_win, self.readme, 1, begin_x)
            begin_y = begin_y + height
            window_list.append(tmp_tuple);

        if 'balloon' in status_list:
            height = 1
            tmp_win = curses.newwin(height, width, begin_y, begin_x)
            tmp_tuple = (tmp_win, self.balloon10, 5, begin_x)
            begin_y = begin_y + height
            window_list.append(tmp_tuple);
            rospy.Subscriber("/" + str(self.UAV_NAME) + "/balloon_circle_destroy/status_out", String, self.balloonCallback)

        if 'dynamics' in status_list:
            height = 3
            tmp_win = curses.newwin(height, width, begin_y, begin_x)
            tmp_tuple = (tmp_win, self.bar10, 5, begin_x)
            begin_y = begin_y + height
            window_list.append(tmp_tuple);

        if 'control_error' in status_list:
            height = 2
            tmp_win = curses.newwin(height, width, begin_y, begin_x)
            tmp_tuple = (tmp_win, self.controlError, 5, begin_x)
            begin_y = begin_y + height
            window_list.append(tmp_tuple);
            rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/control_error", ControlError, self.controlErrorCallback)

        if 'avoidance' in status_list:
            height = 3
            tmp_win = curses.newwin(height, width, begin_y, begin_x)
            tmp_tuple = (tmp_win, self.mpcStatus, 1, begin_x)
            begin_y = begin_y + height
            window_list.append(tmp_tuple);
            rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/mpc_tracker/diagnostics", MpcTrackerDiagnostics, self.MPCstatusCallback)
    

        if 'gripper' in status_list:
            height = 1
            tmp_win = curses.newwin(height, width, begin_y, begin_x)
            tmp_tuple = (tmp_win, self.gripperWin, 1, begin_x)
            begin_y = begin_y + height
            window_list.append(tmp_tuple);
            rospy.Subscriber("/" + str(self.UAV_NAME) + "/gripper/gripper_diagnostics", GripperDiagnostics, self.gripperCallback)

        self.path = "/tmp/mrs_status_time_ " + str(self.UAV_NAME) + ".txt";

        if os.path.isfile(self.path):
            try:
                self.time_file = open(self.path,"r+") 
                self.f_time = int(self.time_file.read())
                self.time_file.close()
            except:
                self.f_time = 0
            os.remove(self.path)

        self.time_file = open(self.path,"w+") 


        rate = rospy.Rate(5.1)
        # time.sleep(1)
    
        # for topics from config list
        self.max_length = 0
    
        # disk space 
        self.last_remaining = 0;
        for i in range(0, len(self.param_list)):
            self.max_length = len(max(self.name_list, key=len))
        self.spacer_string = "          "
        for i in range(0, self.max_length):
            self.spacer_string = self.spacer_string + " "

        global green
        global red
        global white
        global yellow

        if(dark_mode):
            if(colorblind_mode):
                green = curses.color_pair(34)
            else:
                green = curses.color_pair(47)
            yellow = curses.color_pair(221)
            red = curses.color_pair(197)
        else:
            if(colorblind_mode):
                green = curses.color_pair(20)
            else:
                green = curses.color_pair(29)
            yellow = curses.color_pair(173)
            red = curses.color_pair(125)
        white = curses.color_pair(0)
        tmp_color = white
        process.start()
    # #} end of status
            
        loop_counter = 4
        refresh = 0
        self.c_odom = 0
        self.odom_color = red
        width = 180
        ses_name = ""

        for item in window_list:
            if(not dark_mode):
                item[0].attron(curses.A_REVERSE)
            item[0].attron(curses.A_BOLD)

        while not rospy.is_shutdown():
            loop_counter = loop_counter + 1;

            try:
                bashCommand = "tmux display-message -p #S"
                sprocess = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
                output, error = sprocess.communicate()
                ses_name = output;
            except:
                width = 180
                ses_name = ""
            if(ses_name != ""):
                try:
                    bashCommand = "tmux display -p -t " + str(ses_name) + " #{pane_width}"
                    sprocess = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
                    output, error = sprocess.communicate()
                    width = int(output)
                except:
                    width = 180

            if loop_counter == 5:

                loop_counter = 0;  
                refresh = 1;

                stdscr.attron(curses.A_BOLD)

                self.c_odom = self.count_uav_state
                self.count_uav_state = 0

            if refresh == 1:
                refresh = 0;


                # self.uvdarStatus(stdscr)


                for item in window_list:
                    # if item[2] == 1:
                    item[0].clear()
                    if not item[3] > width:
                        item[1](item[0])
                for item in window_list:
                    # if item[2] == 1:
                    item[0].refresh()

                curses.resizeterm(30, width)

            else:

                for item in window_list:
                    if item[2] == 5:
                        item[0].clear()
                        if not item[3] > width:
                            item[1](item[0])
                for item in window_list:
                    if item[2] == 5:
                        item[0].refresh()

            rate.sleep()
            
    # #{ winDisk()

    def winDisk(self, win):
        # win.attroff(green)
        # try:
        #     win.addstr(0, 46, " " + str(os.readlink(str(self.rospack.get_path('mrs_general')) +"/config/world_current.yaml")) + " ")
        # except:
        #     win.attron(red)
        #     win.attron(curses.A_BLINK)
        #     win.addstr(0, 46," NO ARENA DEFINED! ")
        #     win.attroff(curses.A_BLINK)
        #     win.attroff(red)

        # try:
        bashCommand = "df -h"
        p1 = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = p1.communicate()
        output = [line for line in output.split('\n') if line[-1:]=="/" in line]
        output = output[0].split()[3]

        remaining = float(output[:-1].replace(",",".")) 

        tmp_color = green
        if remaining > 25:
            if not remaining == self.last_remaining:
                tmp_color = yellow
            else:
                tmp_color = green
        elif remaining > 10:
            tmp_color = yellow
        else:
            tmp_color = red
            win.attron(curses.A_BLINK)
        self.last_remaining = remaining 
        win.attron(tmp_color)
        win.addstr(0, 0, " Disk space: " + str(output) + " ")
        win.attroff(curses.A_BLINK)
        # except:
        #     win.attron(red)
        #     win.addstr(0, 0, " Disk space: N/A ")

    # #} end of winDisk()
            
    # #{ readme()

    def readme(self, win):

        win.addstr(1, 0, " Mrs status can display a lot more cool stuff. ")
        win.addstr(2, 0, " Just check the MRS_STATUS variable in .bashrc/.zshrc. ")
        win.addstr(3, 0, " Remove the readme entry there to remove this message. ")

    # #} end of readme()
            
    # #{ balloon10()

    def balloon10(self, win):

        status = "NO_DATA"
        if self.count_balloon == 0:
            win.attron(red)
        else:
            status = self.balloon.data
            win.attron(green)
        self.count_balloon = 0
        win.addstr(0, 0, " State: " + str(status) + " ")
        win.attroff(red)
        win.attroff(green)

    # #} end of balloon10()
            
    # #{ controlError()

    def controlError(self, win):

        status = " NO_DATA "
        c_err = 0
        if self.count_control_error == 0:
            win.attron(red)
        else:
            status = " "
            win.attron(white)
            c_err = (self.control_error.total_position_error/self.control_error.position_eland_threshold)
            
        self.count_control_error = 0

        c_bars = int((c_err)*50)

        win.addstr(0, 23, " Control Error:" + str(status))
        win.addstr(1, 0," " +  str(round((self.control_error.total_position_error),1)))
        win.addstr(1, 4, " [")
        win.addstr(1, 56, "] ")
        win.addstr(1, 57, " " + str(round(self.control_error.position_eland_threshold,1)) + " ")

        if c_bars > 50:
            c_bars = 50

        win.attron(green)
        for i in range(0, int(c_bars)):
            if i > 25 and i < 40:
                win.attron(yellow)
            if i > 40:
                win.attron(red)
            win.addstr(1, 6+i, "|")

        win.attroff(red)
        win.attroff(yellow)
        win.attroff(green)

    # #} end of controlError()
            
    # #{ bar10()

    def bar10(self, win):
        if self.count_curr_const > 0:
            self.count_curr_const = 0
            vlim_x = self.curr_const.horizontal_speed
            vlim_y = self.curr_const.horizontal_speed
            vlim_z = self.curr_const.vertical_ascending_speed
            acclim_x = 2.0
            acclim_y = 2.0
            acclim_z = 2.0
        else:
            vlim_x = 0.1
            vlim_y = 0.1
            vlim_z = 0.1
            acclim_x = 0.1
            acclim_y = 0.1
            acclim_z = 0.1

        vlim_x = round(vlim_x,1)
        vlim_y = round(vlim_y,1)
        vlim_z = round(vlim_z,1)
        acclim_x = round(acclim_x,1)
        acclim_y = round(acclim_y,1)
        acclim_z = round(acclim_z,1)

        xv_bars = int(abs(10*self.uav_state.velocity.linear.x/vlim_x))
        if xv_bars > 10:
            xv_bars = 10

        yv_bars = int(abs(10*self.uav_state.velocity.linear.y/vlim_y))
        if yv_bars > 10:
            yv_bars = 10

        zv_bars = int(abs(10*self.uav_state.velocity.linear.z/vlim_z))
        if zv_bars > 10:
            zv_bars = 10

        xa_bars = int(abs(10*self.uav_state.acceleration.linear.x/acclim_x))
        if xa_bars > 10:
            xa_bars = 10

        ya_bars = int(abs(10*self.uav_state.acceleration.linear.y/acclim_y))
        if ya_bars > 10:
            ya_bars = 10

        za_bars = int(abs(10*self.uav_state.acceleration.linear.z/acclim_z))
        if za_bars > 10:
            za_bars = 10

        win.attron(white)

        win.addstr(0, 9, " X: ")
        win.addstr(0, 29, " Y: ")
        win.addstr(0, 49, " Z: ")

        win.addstr(1, 0, " Vel [")
        win.addstr(1, 16, "] " + str(vlim_x))
        win.addstr(1, 24, " [")
        win.addstr(1, 36, "] " + str(vlim_y))
        win.addstr(1, 44, " [")
        win.addstr(1, 56, "] " + str(vlim_z)) 

        win.addstr(2, 0, " Acc [")
        win.addstr(2, 16, "] " + str(acclim_x))
        win.addstr(2, 24, " [")
        win.addstr(2, 36, "] " + str(acclim_y))
        win.addstr(2, 44, " [")
        win.addstr(2, 56, "] " + str(acclim_z)) 

        win.attron(green)
        for i in range(0, xv_bars):
            if i > 4 and i < 7:
                win.attron(yellow)
            if i > 7:
                win.attron(red)
            win.addstr(1, 6+i, "|")
        
        win.attron(green)
        for i in range(0, yv_bars):
            if i > 4 and i < 7:
                win.attron(yellow)
            if i > 7:
                win.attron(red)
            win.addstr(1, 26+i, "|")

        win.attron(green)
        for i in range(0, zv_bars):
            if i > 4 and i < 7:
                win.attron(yellow)
            if i > 7:
                win.attron(red)
            win.addstr(1, 46+i, "|")

        win.attron(green)
        for i in range(0, xa_bars):
            if i > 4 and i < 7:
                win.attron(yellow)
            if i > 7:
                win.attron(red)
            win.addstr(2, 6+i, "|")
        
        win.attron(green)
        for i in range(0, ya_bars):
            if i > 4 and i < 7:
                win.attron(yellow)
            if i > 7:
                win.attron(red)
            win.addstr(2, 26+i, "|")

        win.attron(green)
        for i in range(0, za_bars):
            if i > 4 and i < 7:
                win.attron(yellow)
            if i > 7:
                win.attron(red)
            win.addstr(2, 46+i, "|")
        win.attroff(green)
        win.attroff(red)
        win.attroff(yellow)

    # #} end of bar10()

    # #{ namesWin()
    
    def namesWin(self, win):
        win.attron(white)
        win.addstr(0, 0," " + str(self.UAV_NAME) + " ")
        win.addstr(0, 6," " + str(self.UAV_TYPE) + " ")
        win.addstr(0, 12," " + str(self.NATO_NAME) + " ")
        win.attroff(white)
    
    # #} end of namesWin()

    # #{ timeWin()
    
    def timeWin(self, win):
        print_time = ""
        if self.first_run:
            self.current_time = int(self.f_time)
            self.first_run = False
        if  self.first_tracker_flag and self.tracker_flag:
            self.start_time = rospy.Time.now(); 
            self.first_tracker_flag = False
        if self.tracker_flag:
            self.current_time = (rospy.Time.now().secs - self.start_time.secs) + int(self.f_time)
            os.remove(self.path)
            self.time_file = open(self.path,"w+") 
            self.time_file.write(str(self.current_time)) 
            self.time_file.close() 
        mins = int(self.current_time/60)
        secs = int(self.current_time%60)
        if secs < 10:
            space = "0"
        else:
            space = ""
        win.addstr(0, 0," Flight time: " + str(mins) + ":" + space + str(secs) + " ")

    # #} end of timeWin()

    # #{ controlWin()
    
    def controlWin(self, win):
    
        # #{ Controller
    
        tmp = self.count_controller
        self.count_controller = 0
        if tmp == 0:
            controller = "NO CONTROLLER"
            tmp_color = red
        else:
            controller = self.controller_status.controller.rsplit('/', 1)[-1]
    
        if controller == "So3Controller" or controller == "MpcController":
            tmp_color = green
        else:
            tmp_color = red
    
        win.attron(tmp_color)
        win.addstr(0, 0, " " + controller + " ")
    
        tmp_offset = len(controller) + 1
    
        if controller == "So3Controller":
          tmp = self.count_gains
          self.count_gains = 0
          if tmp == 0:
              cur_gains = "N/A"
              tmp_color = red
          else:
              cur_gains = self.gains.current_name
    
          if cur_gains == "soft":
              tmp_color = green
          elif cur_gains == "supersoft" or cur_gains == "tight":
              tmp_color = yellow
          else:
              tmp_color = red
    
          win.attroff(tmp_color)
          win.addstr(0, tmp_offset, "/")
          win.attron(tmp_color)
          win.addstr(0, tmp_offset + 1, cur_gains + " ")
    
        # #} end of Controller
    
        # #{ Tracker
    
        tmp = self.count_tracker
        self.count_tracker = 0
        if tmp == 0:
            self.tracker = "NO TRACKER"
            tmp_color = red
            self.tracker_flag = False
        else:
            self.tracker = self.tracker_status.tracker.rsplit('/', 1)[-1]
            if self.tracker == "NullTracker":
                self.tracker_flag = False
            else:
                self.tracker_flag = True
    
        if self.tracker == "MpcTracker":
            tmp_color = green
        elif self.tracker == "LineTracker" or self.tracker == "LandoffTracker":
            tmp_color = yellow
        else:
            tmp_color = red

        win.attron(tmp_color)
        win.addstr(1, 0, " " + self.tracker + " ")
    
        tmp_offset = len(self.tracker) + 1
    
        tmp = self.count_constraints
        self.count_constraints = 0
        if tmp == 0:
            cur_constraints= "N/A"
            tmp_color = red
        else:
            cur_constraints = self.constraints.current_name
    
        if cur_constraints == "medium":
            tmp_color = green
        elif cur_constraints == "slow" or cur_constraints == "fast" or cur_constraints == "optflow":
            tmp_color = yellow
        else:
            tmp_color = red
    
        win.attroff(tmp_color)
        win.addstr(1, tmp_offset, "/")
        win.attron(tmp_color)
        win.addstr(1, tmp_offset + 1, cur_constraints + " ")
        win.attroff(tmp_color)
    
        # #} end of Tracker
    
        # #{ Thrust
    
        tmp = self.count_attitude
        if tmp == 0:
            thrust = "N/A"
            tmp_color = red
        else:
            thrust = round(self.attitude_cmd.thrust, 3)
            tmp_color = green
            if thrust > 0.7 or thrust < 0.25:
                tmp_color = yellow
            if thrust > 0.79:
                tmp_color = red
        win.attron(tmp_color)
        win.addstr(2, 0, " Thrust: " + str(thrust) + " ")
        win.attroff(white)
        win.attroff(red)
        win.attroff(green)
        win.attroff(yellow)
    
        # #} end of Thrust
    
    # #} end of controlWin()
            
    # #{ odomWin()

    def odomWin(self, win):

        self.odom = ""
        self.odom_color = green
        count = self.c_odom;
        if self.c_odom == 0:
            count = "NO_ODOM"
            self.odom_color = red
        else:
            self.odom = self.uav_state.header.frame_id.split("/")[1]
            if self.c_odom < 0.9*100 or self.c_odom > 1.1*100:
                self.odom_color = yellow
            win.attron(self.odom_color)
            win.addstr(0, 11, " Odom     Hz ")

        win.attron(self.odom_color)
        win.addstr(0, 15 + (5 - len(str(count))),str(count) + " ")
        win.addstr(1, 11, " " + str(self.odom) + " ")

        win.attron(self.odom_color)
        
        tmp = round(self.uav_state.pose.position.x,2)
        if len(str(tmp).split('.')[1]) == 1:
            tmp = str(tmp) + "0"
        win.addstr(0, 0, unicode(" X    "))
        win.addstr(0, 6-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

        tmp = round(self.uav_state.pose.position.y,2)
        if len(str(tmp).split('.')[1]) == 1:
            tmp = str(tmp) + "0"
        win.addstr(1, 0, " Y    ")
        win.addstr(1, 6-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

        tmp = round(self.uav_state.pose.position.z,2)
        if len(str(tmp).split('.')[1]) == 1:
            tmp = str(tmp) + "0"
        win.addstr(2, 0, " Z    ")
        win.addstr(2, 6-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

        quaternion = (
            self.uav_state.pose.orientation.x,
            self.uav_state.pose.orientation.y,
            self.uav_state.pose.orientation.z,
            self.uav_state.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        tmp = round(euler[2], 2)
        win.addstr(2, 11, " yaw     ")
        win.addstr(2, 17 - (len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

        win.addstr(0, 26, " Hori: " + str(self.uav_state.estimator_horizontal.name) + " ")
        win.addstr(1, 26, " Vert: " + str(self.uav_state.estimator_vertical.name) + " ")
        win.addstr(2, 26, " Head: " + str(self.uav_state.estimator_heading.name) + " ")

        win.attroff(white)
        win.attroff(red)
        win.attroff(green)
        win.attroff(yellow)

    # #} end of odomWin()
            
    # #{ bumperStatus()

    def bumperStatus(self, stdscr):
        if self.bumper_status.repulsing or self.bumper_status.modifying_reference:
            tmp_color = red
        else:
            tmp_color = green
        stdscr.attron(tmp_color)
        stdscr.addstr(4, 78 + self.max_length, " BUMPER: ")
        if self.bumper_status.repulsing:
            stdscr.addstr(5, 79 + self.max_length, " REPULSING ")
        elif self.bumper_status.modifying_reference:
            stdscr.addstr(5, 79 + self.max_length, " MODIFYING ")
        else:
            stdscr.addstr(5, 79 + self.max_length, " no msgs ")
        self.bumper_status.repulsing = False
        self.bumper_status.modifying_reference = False

    # #} end of bumperStatus()
            
    # #{ uvdarStatus()

    def uvdarStatus(self, stdscr):
        if 'uvdar' in self.sensor_list:
            tmp = self.count_uvdar
            self.count_uvdar = 0
            if tmp == 0:
                uvdar_text = "no msgs"
                tmp_color = red
            else:
                uvdar_text = str(self.uvdar.data)
                uvdar_text = uvdar_text + " targets"
                tmp_color = green
                if self.uvdar.data == 0:
                    tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(1, 78 + self.max_length, " UVDAR sees: ")
            stdscr.addstr(2, 79 + self.max_length, " " + uvdar_text + " ")

    # #} end of uvdarStatus()
            
    # #{ gripperWin()

    def gripperWin(self, win):
        tmp = self.count_gripper
        status = ""
        if tmp == 0 :
            tmp_color = red
            status = "NO_DATA"
        else:
            tmp_color = green
        win.attron(tmp_color)

        win.addstr(0, 0 , " Gripper: " + str(status))

        if(self.count_gripper > 0):
            win.attron(red)
            if str(self.gripper.gripper_on) == "True":
                win.attron(green)
            win.addstr(0, 15 , " On: " + str(self.gripper.gripper_on) + " ")
            win.attron(red)
            if str(self.gripper.gripping_object) == "True":
                win.attron(green)
            win.addstr(0, 30 , " Has Object: " + str(self.gripper.gripping_object) + " ")

        win.attroff(red)
        win.attroff(green)
        

        self.count_gripper = 0

    # #} end of gripperWin()

# #{ mpcStatus()

    def mpcStatus(self, win):
        tmp = self.count_mpcstatus
        if tmp == 0 :
            tmp_color = red
        elif self.mpcstatus.collision_avoidance_active != True:
            tmp_color = red
        else:
            tmp_color = green
        win.attron(tmp_color)
        if self.count_mpcstatus > 0:
            win.addstr(0, 0 , " C.Avoid: " + str(self.mpcstatus.collision_avoidance_active))
            win.addstr(1, 0 , " I see: " + str(self.mpcstatus.avoidance_active_uavs))
            tmp_color = red
            win.attron(tmp_color)
            if self.mpcstatus.avoiding_collision == True:
                win.addstr(2, 1, " ! AVOIDING COLLISION ! ")
        else:
            tmp_color = red
            win.attron(tmp_color)
            win.addstr(0, 0, " C.Avoid: NO DATA ")
            win.attroff(tmp_color)

        self.count_mpcstatus = 0

    # #} end of mpcStatus()
            
    # #{ winGps()

    def winGps(self, win):

        # #{ GPS
        
        if "GPS" in str(self.odom) or  "Gps" in str(self.odom) or "gps" in str(self.odom) or self.has_gps:
            tmp = self.count_gpsdata
            self.count_gpsdata = 0
            if tmp == 0:
                gps = ""
            else:
                gps = (self.gpsdata.position_covariance[0] + self.gpsdata.position_covariance[4] + self.gpsdata.position_covariance[8])/3
                gps = round(gps,2)
                tmp_color = green
                if float(gps) < 10.0:
                    tmp_color = green
                elif float(gps) < 20.0:
                    tmp_color = yellow
                else:
                    win.attron(curses.A_BLINK)
                    tmp_color = red
            win.attron(tmp_color)
            win.addstr(0, 0, " GPS qual: " + str(gps) + " ")
            win.attroff(curses.A_BLINK)
        
        # #} end of GPS

        # #{ RTK
        
        tmp = self.count_bestpos
        self.count_bestpos = 0
        if tmp == 0:
            rtk = "no msgs"
            tmp_color = red
        else:
            rtk = self.bestpos.position_type
            tmp_color = green
            if rtk == "L1_INT" or rtk == "WIDE_INT" or rtk == "NARROW_INT":
                tmp_color = green
            elif rtk == "L1_FLOAT" or rtk == "NARROW_FLOAT":
                tmp_color = yellow
            else:
                tmp_color = red
        win.attron(tmp_color)
        win.addstr(1, 0, " RTK: " + str(rtk) + " ")
        
        # #} end of RTK

    # #} end of  winGps()

    # #{ winConfTopics()

    def winConfTopics(self, win):
        for i in range(0, len(self.param_list)):
            if(self.count_list[i] > 0):
                tmp = self.count_list[i]
                self.count_list[i] = 0
            else:
                tmp = 0

            tmp_color = green
            if tmp == 0:
                tmp_color = red
            else:
                if tmp < 0.9*self.hz_list[i] or tmp > 1.1*self.hz_list[i]:
                    tmp_color = yellow
                if tmp > 1.1*self.hz_list[i] and self.hz_list_modifiers[i] == "+":
                    tmp_color = green
                if tmp < 0.9*self.hz_list[i] and self.hz_list_modifiers[i] == "-":
                    tmp_color = green
            win.attron(tmp_color)

            if self.many_param_flag:
                if i < 7:
                    win.addstr(0 + i, 0, self.spacer_string)
                    win.addstr(0 + i, 0, " " + str(self.name_list[i]) + ": ")
                    win.addstr(0 + i, 0 + self.max_length + (5 - len(str(tmp))), " " + str(tmp) + " Hz ")
                else:
                    win.addstr(0 + i - 7, 17, self.spacer_string)
                    win.addstr(0 + i - 7, 17, " " + str(self.name_list[i]) + ": ")
                    win.addstr(0 + i - 7, 17 + self.max_length + (5 - len(str(tmp))), " " + str(tmp) + " Hz ")

            else:
                win.addstr(0 + i, 0, self.spacer_string)
                win.addstr(0 + i, 0, " " + str(self.name_list[i]) + ": ")
                win.addstr(0 + i, 0 + self.max_length + 2 + (5 - len(str(tmp))), " " + str(tmp) + " Hz ")

    # #} winConfTopics()

    # #{ massWin()
                
    def massWin(self, win):
        tmp_color = white
        win.attron(tmp_color)
        set_mass = float(self.UAV_MASS.replace(",","."))
        set_mass = round(set_mass, 2)
        if (len(str(set_mass).split('.')[1]) == 1):
            set_mass = str(set_mass) + "0"
        win.addstr(0, 1," " + str(set_mass) + "/")

        est_mass = round(self.attitude_cmd.total_mass, 2)
        tmp_color = green
        if abs(self.attitude_cmd.mass_difference) > 2.0:
            tmp_color = red
            win.attron(curses.A_BLINK)
        elif abs(self.attitude_cmd.mass_difference) > 1.0:
           tmp_color = yellow
        
        win.attron(tmp_color)
       
        if self.count_attitude > 0:
            self.count_attitude = 0
            if (len(str(est_mass).split('.')[1]) == 1):
                est_mass = str(est_mass) + "0"
            win.addstr(0, 3 + len(str(set_mass)), str(est_mass))
            win.attroff(tmp_color)
            win.addstr(0, 3 + len(str(set_mass)) + len(str(est_mass)), " kg ")
        else:
            tmp_color = red
            win.attron(tmp_color)
            win.addstr(0, 3 + len(str(set_mass))," N/A kg ")
        
        win.attroff(curses.A_BLINK)
        win.attroff(tmp_color)
                
    # #} end of massWin()

    # #{ pixhawkWin()

    def pixhawkWin(self, win):

        # #{ Armed, Offboard
        
        tmp = self.count_state
        self.count_state = 0
        tmp2 = "N/A"
        if tmp == 0:
            tmp = "N/A"
        else:
            tmp = self.mavros_state.armed
            tmp2 = self.mavros_state.mode
        if tmp == True:
            tmp = "ARMED"
            tmp_color = green
        else:
            tmp = "DISARMED"
            tmp_color = red
        win.attron(tmp_color)
        win.addstr(0, 0, " State: " + str(tmp) + " ")
        if(str(tmp2) == "OFFBOARD"):
            tmp_color = green
        elif(str(tmp2) == "POSITION" or str(tmp2) == "MANUAL" or str(tmp2) == "ALTITUDE" ):
            tmp_color = yellow
        else:
            tmp_color = red
        win.attron(tmp_color)
        win.addstr(1, 0, " Mode:  " + str(tmp2) + " ")
        win.attroff(tmp_color)
        
        # #} end of Armed, Offboard

        # #{ Battery
        
        tmp = self.count_battery
        self.count_battery = 0
        if tmp == 0 and self.last_battery == "N/A ":
            bat_v_out = "N/A "
            bat_a_out = "N/A "
            tmp_color = red
        else:
            if tmp == 0:
                battery = self.last_battery
                current = self.last_current
                self.last_battery = "N/A "
                self.last_current = "N/A "
            else:
                battery = self.battery.voltage
                current = self.battery.current
                self.last_battery = self.battery.voltage
                self.last_current = self.battery.current
            if battery > 17.0:
                bat_v_out = battery/6
            else:
                bat_v_out = battery/4
            bat_v_out = round(bat_v_out, 2)
            bat_a_out = round(current, 1)
            tmp_color = green
            if (bat_v_out > 3.7):
                tmp_color = green
            elif (bat_v_out > 3.55):
                tmp_color = yellow
            else:
                win.attron(curses.A_BLINK)
                tmp_color = red
        win.attron(tmp_color)
        win.addstr(2, 0, " " + str(bat_v_out) + " ")
        win.addstr(2, 6, "V ")
        # bat_a_out = abs(bat_a_out)
        if bat_a_out >= 30 and bat_a_out < 50:
            win.attron(yellow)
        if bat_a_out >= 50:
            win.attron(red)
        win.addstr(2, 10 - (len(str(bat_a_out).split('.')[0])), " " + str(bat_a_out) + " ")
        win.addstr(2, 14, "A ")
        win.attroff(curses.A_BLINK)
        
        # #} end of Battery
        
        # #} end of pixhawkWin()

    # #{ cpuLoadWin()
    
    def cpuLoadWin(self, win):
        cpu_load = 0
        tmp_color = green
        if process.is_alive():
            queue_lock.acquire()
            if not cpu_load_queue.empty():
                cpu_load = cpu_load_queue.get_nowait()
                while not cpu_load_queue.empty():
                    cpu_load_queue.get_nowait()
            queue_lock.release()
            if(int(cpu_load) > 89):
                tmp_color = red
            elif(int(cpu_load) > 74):
                tmp_color = yellow
        else:
            tmp_color = red
            cpu_load = "x"
        win.attron(tmp_color)
        win.addstr(0, 0, " CPU load:   ")
        win.addstr(0, 9 + (4 - len(str(cpu_load))), str(cpu_load))
        win.addstr(0, 13, " % ")
        win.attroff(tmp_color)
    
    # #} end of cpuLoadWin

    def __init__(self):

        # #{ Var definitions
        
        self.tracker_status = TrackerStatus()
        self.controller_status = ControllerStatus()
        self.uav_state = UavState()
        self.mavros_state = State()
        self.attitude_cmd = AttitudeCommand()
        self.gains = GainManagerDiagnostics()
        self.constraints = ConstraintManagerDiagnostics()
        self.bestpos = Bestpos()
        self.battery = BatteryState()
        self.gpsdata = NavSatFix()
        self.bumper_status = BumperStatus()
        self.curr_const = TrackerConstraints()
        self.balloon = String()
        self.control_error = ControlError()
        self.mpcstatus = MpcTrackerDiagnostics()
        self.gripper = GripperDiagnostics()

        self.uvdar = Int16()
        self.count_list = []
        self.count_uav_state = 0
        self.count_state = 0
        self.count_attitude_target = 0
        self.count_attitude = 0
        self.count_controller = 0
        self.count_tracker = 0
        self.count_gains = 0
        self.count_constraints = 0
        self.count_bestpos = 0
        self.count_battery = 0
        self.count_mpcstatus = 0
        self.count_gpsdata = 0
        self.count_uvdar = 0
        self.count_curr_const = 0
        self.count_balloon = 0
        self.count_control_error = 0
        self.count_gripper = 0

        self.last_battery = 0
        self.last_current = 0

        self.bumper_repulsing = 0
        self.bumper_constraining = 0
        self.has_gps = False
        self.rospack = rospkg.RosPack()

        self.f_time = 0; 
        self.start_time = 0;
        self.current_time = 0;

        self.first_run = True
        self.first_tracker_flag = True
        self.tracker_flag = False
        self.many_param_flag = False

        try:
            self.UAV_NAME =str(os.environ["UAV_NAME"])
        except:
            self.ErrorShutdown(" UAV_NAME variable is not set!!! Terminating... ", stdscr, red)
        
        try:
            self.UAV_MASS =str(os.environ["UAV_MASS"])
        except:
            self.ErrorShutdown(" UAV_MASS variable is not set!!! Terminating... ", stdscr, red)
        
        try:
            self.UAV_TYPE =str(os.environ["UAV_TYPE"])
        except:
            self.ErrorShutdown(" UAV_TYPE variable is not set!!! Terminating... ", stdscr, red)

        try:
            self.ODOMETRY_TYPE =str(os.environ["ODOMETRY_TYPE"])
        except:
            self.ErrorShutdown(" ODOMETRY_TYPE variable is not set!!! Terminating... ", stdscr, red)

        try:
            self.PIXGARM =str(os.environ["PIXGARM"])
        except:
            self.PIXGARM ="false"
        
        try:
            self.NATO_NAME =str(os.environ["NATO_NAME"])
        except:
            self.NATO_NAME =""
        
        try:
            self.SENSORS =str(os.environ["SENSORS"])
        except:
            self.SENSORS =""

        try:
            self.MRS_STATUS =str(os.environ["MRS_STATUS"])
        except:
            self.MRS_STATUS =""

        try:
            self.BLUEFOX_UV_LEFT =str(os.environ["BLUEFOX_UV_LEFT"])
        except:
            self.BLUEFOX_UV_LEFT =""
        
        try:
            self.BLUEFOX_UV_RIGHT =str(os.environ["BLUEFOX_UV_RIGHT"])
        except:
            self.BLUEFOX_UV_RIGHT =""

        try:
            self.PROFILES_BOTH =str(os.environ["PROFILES_BOTH"])
        except:
            self.PROFILES_BOTH =""

        # #} end of Var definitions
        curses.wrapper(self.status)

if __name__ == '__main__':
    rospy.init_node('status', anonymous=True)
    try:
        status = Status()
    except rospy.ROSInterruptException:
        pass

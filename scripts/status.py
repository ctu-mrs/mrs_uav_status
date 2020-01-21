#!/usr/bin/env python
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

    def AttitudeTargetCallback(self, data):
        self.attitude_target = data
        self.count_attitude_target = self.count_attitude_target + 1

    def BestposCallback(self, data):
        self.bestpos = data
        self.count_bestpos = self.count_bestpos + 1

    def BatteryCallback(self, data):
        self.battery = data
        self.count_battery = self.count_battery + 1

    def MPCstatusCallback(self, data):
        self.mpcstatus = data
        self.count_mpcstatus = self.count_mpcstatus + 1

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

        rospy.init_node('status', anonymous=True)

        self.UAV_MASS = "{}".format(float(rospy.get_param("~uav_mass")))

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

        self.param_list = rospy.get_param('~want_hz', "")
        self.SENSORS = str(self.SENSORS).replace(',', ' ') 
        self.sensor_list = str(self.SENSORS).split(' ')
        # needed_nodes = rospy.get_param('~needed_nodes', "")
        # for i in needed_nodes:
        #     i = str(self.UAV_NAME) + "/" + i

        if str(self.PIXGARM) == "true":
            self.param_list.insert(0, "mavros/distance_sensor/garmin Garmin_pix 80+")

        if str(self.PIXGARM) == "false" and 'garmin_down' in self.sensor_list:
            self.param_list.insert(0, "garmin/range Garmin_down 80+")

        if 'realsense_brick' in self.sensor_list:
            self.param_list.insert(0, "rs_d435/color/image_raw Realsense_Brick 25+")

        if 'bluefox_brick' in self.sensor_list:
            self.param_list.insert(0, "bluefox_brick/image_raw Bluefox_Brick 25+")

        if 'bluefox_optflow' in self.sensor_list:
            self.param_list.insert(0, "bluefox_optflow/image_raw Bluefox_Optflow 60+")
            self.param_list.insert(0, "optic_flow/velocity Optic_flow 60+")

        if 'rplidar' in self.sensor_list:
            self.param_list.insert(0, "rplidar/scan Rplidar 10+")

        if str(self.BLUEFOX_UV_LEFT) != "":
            self.param_list.insert(0, "uvdar_bluefox/left/image_raw Bluefox_UV_left 70+")

        if str(self.BLUEFOX_UV_RIGHT) != "":
            self.param_list.insert(0, "uvdar_bluefox/right/image_raw Bluefox_UV_right 70+")

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
        profile_list = str(self.PROFILES_BOTH).split(' ')
        dark_mode = False
        if 'COLORSCHEME_DARK' in profile_list:
            dark_mode = True

        # dark_mode = rospy.get_param('~dark_mode', True)
        colorblind_mode = rospy.get_param('~colorblind_mode', True)

        stdscr.addstr(10, 40," " + str(tmp_string))

        # Create ROSTopicHz and subscribers defined by the loaded config
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/diagnostics", ControlManagerDiagnostics, self.ControlManagerDiagnostics)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/attitude_cmd", AttitudeCommand, self.AttitudeCmdCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/odometry/uav_state", UavState, self.UavStateCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/state", State, self.StateCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.AttitudeTargetCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/gain_manager/current_gains", String, self.GainsCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/constraint_manager/current_constraints", String, self.ConstraintsCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/tersus/bestpos", Bestpos, self.BestposCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/battery", BatteryState, self.BatteryCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/mpc_tracker/diagnostics", MpcTrackerDiagnostics, self.MPCstatusCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/global_position/global", NavSatFix, self.GPSCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/bumper_status", BumperStatus, self.bumperCallback)
        if 'uvdar' in self.sensor_list:
            rospy.Subscriber("/" + str(self.UAV_NAME) + "/uvdar/targetSeenCount", Int16, self.uvdarCallback)
    
        for i in range(0, len(self.param_list)):
            if(str(topic_list[i][0]) == "/"):
                sub_list.append(rospy.Subscriber(str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            else:
                sub_list.append(rospy.Subscriber("/" + str(self.UAV_NAME) + "/" + str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            self.count_list.append(0)
    
        rate = rospy.Rate(10)
        time.sleep(1)
    
        # for topics from config list
        self.max_length = 0
    
        # disk space 
        last_remaining = 0;
        for i in range(0, len(self.param_list)):
            self.max_length = len(max(self.name_list, key=len))
        self.spacer_string = "          "
        for i in range(0, self.max_length):
            self.spacer_string = self.spacer_string + " "
    
        if(dark_mode):
            if(colorblind_mode):
                self.green = curses.color_pair(34)
            else:
                self.green = curses.color_pair(47)
            self.yellow = curses.color_pair(221)
            self.red = curses.color_pair(197)
        else:
            if(colorblind_mode):
                self.green = curses.color_pair(20)
            else:
                self.green = curses.color_pair(29)
            self.yellow = curses.color_pair(173)
            self.red = curses.color_pair(125)
        self.white = curses.color_pair(0)
        tmp_color = curses.color_pair(0)
        process.start()
    # #} end of status
            
        loop_counter = 9
        refresh = 0
        c_odom = 0
        self.odom_color = self.red

        begin_x = 0; begin_y = 5
        height = 3; width = 26
        uav_state_win = curses.newwin(height, width, begin_y, begin_x)
        begin_x = 97; begin_y = 1
        height = 5; width = 60
        bar_win = curses.newwin(height, width, begin_y, begin_x)

        while not rospy.is_shutdown():
            loop_counter = loop_counter + 1;
            curses.resizeterm(30, 180)

            if loop_counter == 10:
                loop_counter = 0;  
                refresh = 1;

                stdscr.attroff(tmp_color)
                if(not dark_mode):
                    stdscr.attron(curses.A_REVERSE)
                stdscr.attron(curses.A_BOLD)
                uav_state_win.attron(curses.A_BOLD)
                bar_win.attron(curses.A_BOLD)

                c_odom = self.count_uav_state
                self.count_uav_state = 0

            uav_state_win.clear()
            bar_win.clear()
            if(not dark_mode):
                uav_state_win.attron(curses.A_REVERSE)
                bar_win.attron(curses.A_REVERSE)
            self.odom10(uav_state_win, c_odom)
            self.bar10(bar_win, c_odom)

            if refresh == 1:
                refresh = 0;

                # #{ Odom Hz
                # #} end of Odom

                stdscr.clear()

                self.cpuLoad(stdscr)
                self.mavrosState(stdscr)
                self.activeTracker(stdscr)
                self.names(stdscr)
                self.mass(stdscr)
                self.thrust(stdscr)
                self.activeController(stdscr)
                self.confTopics(stdscr)
                self.gps(stdscr)
                self.rtk(stdscr)
                self.batteryStatus(stdscr)
                self.mpcStatus(stdscr)
                self.uvdarStatus(stdscr)
                self.bumperStatus(stdscr)
                self.misc(stdscr)

            stdscr.refresh()
            uav_state_win.refresh()
            bar_win.refresh()
            rate.sleep()
            
    # #{ misc()

    def misc(self, stdscr):
        stdscr.attroff(self.green)
        try:
            stdscr.addstr(0, 46, " " + str(os.readlink(str(self.rospack.get_path('mrs_general')) +"/config/world_current.yaml")) + " ")
        except:
            stdscr.attron(self.red)
            stdscr.attron(curses.A_BLINK)
            stdscr.addstr(0, 46," NO ARENA DEFINED! ")
            stdscr.attroff(curses.A_BLINK)
            stdscr.attroff(self.red)
        try:
            bashCommand = "df -h"
            p1 = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
            output, error = p1.communicate()
            output = [line for line in output.split('\n') if line[-1:]=="/" in line]
            output = output[0].split()[3]

            remaining = float(output[:-1].replace(",",".")) 

            stdscr.attroff(tmp_color)
            if remaining > 25:
                if not remaining == last_remaining:
                    tmp_color = self.yellow
                else:
                    tmp_color = self.green
            elif remaining > 10:
                tmp_color = self.yellow
            else:
                tmp_color = self.red
                stdscr.attron(curses.A_BLINK)
            last_remaining = remaining 
            stdscr.attron(tmp_color)
            stdscr.addstr(1, 60 + self.max_length, " Disk space: " + str(output) + " ")
            stdscr.attroff(curses.A_BLINK)
        except:
            stdscr.attron(self.red)
            stdscr.addstr(1, 60 + self.max_length, " Disk space: N/A ")

    # #} end of misc()
            
            
    # #{ bar10()

    def bar10(self, win, c_odom):
        vlim_x = 3
        vlim_y = 3
        vlim_z = 3
        acclim_x = 2.0
        acclim_y = 2.0
        acclim_z = 2.0
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

        win.attron(self.white)
        win.addstr(0, 15 - (len(str(vlim_x).split('.')[0])), " " + str(vlim_x) + " ")
        win.addstr(0, 31 - (len(str(vlim_y).split('.')[0])), " " + str(vlim_y) + " ")
        win.addstr(0, 47 - (len(str(vlim_z).split('.')[0])), " " + str(vlim_z) + " ")
        win.addstr(1, 0, " Vel X:[")
        win.addstr(1, 18, "]  Y:[")
        win.addstr(1, 34, "]  Z:[")
        win.addstr(1, 50, "] ")

        win.addstr(3, 15 - (len(str(acclim_x).split('.')[0])), " " + str(acclim_x) + " ")
        win.addstr(3, 31 - (len(str(acclim_y).split('.')[0])), " " + str(acclim_y) + " ")
        win.addstr(3, 47 - (len(str(acclim_z).split('.')[0])), " " + str(acclim_z) + " ")
        win.addstr(4, 0, " Acc X:[")
        win.addstr(4, 18, "]  Y:[")
        win.addstr(4, 34, "]  Z:[")
        win.addstr(4, 50, "] ")

        win.attron(self.green)
        for i in range(0, xv_bars):
            if i > 4 and i < 7:
                win.attron(self.yellow)
            if i > 7:
                win.attron(self.red)
            win.addstr(1, 8+i, "|")
        
        win.attron(self.green)
        for i in range(0, yv_bars):
            if i > 4 and i < 7:
                win.attron(self.yellow)
            if i > 7:
                win.attron(self.red)
            win.addstr(1, 24+i, "|")

        win.attron(self.green)
        for i in range(0, zv_bars):
            if i > 4 and i < 7:
                win.attron(self.yellow)
            if i > 7:
                win.attron(self.red)
            win.addstr(1, 40+i, "|")

        win.attron(self.green)
        for i in range(0, xa_bars):
            if i > 4 and i < 7:
                win.attron(self.yellow)
            if i > 7:
                win.attron(self.red)
            win.addstr(4, 8+i, "|")
        
        win.attron(self.green)
        for i in range(0, ya_bars):
            if i > 4 and i < 7:
                win.attron(self.yellow)
            if i > 7:
                win.attron(self.red)
            win.addstr(4, 24+i, "|")

        win.attron(self.green)
        for i in range(0, za_bars):
            if i > 4 and i < 7:
                win.attron(self.yellow)
            if i > 7:
                win.attron(self.red)
            win.addstr(4, 40+i, "|")
        win.attroff(self.green)
        win.attroff(self.red)
        win.attroff(self.yellow)

        # win.attroff(self.green)
        # self.odom = ""
        # self.odom_color = self.green
        # if c_odom == 0:
        #     c_odom = "NO_ODOM"
        #     self.odom_color = self.red
        # else:
        #     self.odom = self.uav_state.header.frame_id.split("/")[1]
        #     if c_odom < 0.9*100 or c_odom > 1.1*100:
        #         self.odom_color = self.yellow
        # win.attron(self.odom_color)
        # win.addstr(0, 0, " Odom:     Hz ")
        # win.addstr(0, 5 + (5 - len(str(c_odom))),str(c_odom) + " ")
        # win.addstr(1, 0, " " + str(self.odom) + " ")

        # win.addstr(1, 0, " ")
        # win.addstr(2, 0, " ")
        # win.attron(self.odom_color)
        # tmp = round(self.uav_state.pose.pose.position.x,2)
        # win.addstr(0, 14, "       ")
        # win.addstr(0, 21, " X ")
        # win.addstr(0, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

        # if len(str(tmp).split('.')[1]) == 1:
        #     win.addstr(0, 20, "0")
        # tmp = round(self.uav_state.pose.pose.position.y,2)
        # win.addstr(1, 14, "       ")
        # win.addstr(1, 21, " Y ")
        # win.addstr(1, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
        # if len(str(tmp).split('.')[1]) == 1:
        #     win.addstr(1, 20, "0")

        # tmp = round(self.uav_state.pose.pose.position.z,2)
        # win.addstr(2, 14, "       ")
        # win.addstr(2, 21, " Z ")
        # win.addstr(2, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
        # if len(str(tmp).split('.')[1]) == 1:
        #     win.addstr(2, 20, "0")


    # #} end of bar10()
            
    # #{ odom10()

    def odom10(self, win, c_odom):

        self.odom = ""
        self.odom_color = self.green
        if c_odom == 0:
            c_odom = "NO_ODOM"
            self.odom_color = self.red
        else:
            self.odom = self.uav_state.header.frame_id.split("/")[1]
            if c_odom < 0.9*100 or c_odom > 1.1*100:
                self.odom_color = self.yellow
        win.attron(self.odom_color)
        win.addstr(0, 0, " Odom:     Hz ")
        win.addstr(0, 5 + (5 - len(str(c_odom))),str(c_odom) + " ")
        win.addstr(1, 0, " " + str(self.odom) + " ")

        win.addstr(1, 0, " ")
        win.addstr(2, 0, " ")
        win.attron(self.odom_color)
        tmp = round(self.uav_state.pose.position.x,2)
        win.addstr(0, 14, "       ")
        win.addstr(0, 21, " X ")
        win.addstr(0, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

        if len(str(tmp).split('.')[1]) == 1:
            win.addstr(0, 20, "0")
        tmp = round(self.uav_state.pose.position.y,2)
        win.addstr(1, 14, "       ")
        win.addstr(1, 21, " Y ")
        win.addstr(1, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
        if len(str(tmp).split('.')[1]) == 1:
            win.addstr(1, 20, "0")

        tmp = round(self.uav_state.pose.position.z,2)
        win.addstr(2, 14, "       ")
        win.addstr(2, 21, " Z ")
        win.addstr(2, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
        if len(str(tmp).split('.')[1]) == 1:
            win.addstr(2, 20, "0")

        quaternion = (
            self.uav_state.pose.orientation.x,
            self.uav_state.pose.orientation.y,
            self.uav_state.pose.orientation.z,
            self.uav_state.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        tmp = round(euler[2], 2)
        win.addstr(2, 4, "       ")
        win.addstr(2, 9, " yaw ")
        win.addstr(2, 5 - (len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

    # #} end of odom10()
            
    # #{ bumperStatus()

    def bumperStatus(self, stdscr):
        if self.bumper_status.repulsing or self.bumper_status.modifying_reference:
            tmp_color = self.red
        else:
            tmp_color = self.green
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
                tmp_color = self.red
            else:
                uvdar_text = str(self.uvdar.data)
                uvdar_text = uvdar_text + " targets"
                tmp_color = self.green
                if self.uvdar.data == 0:
                    tmp_color = self.red
            stdscr.attron(tmp_color)
            stdscr.addstr(1, 78 + self.max_length, " UVDAR sees: ")
            stdscr.addstr(2, 79 + self.max_length, " " + uvdar_text + " ")

    # #} end of uvdarStatus()
            
    # #{ mpcStatus()

    def mpcStatus(self, stdscr):
        if self.tracker == "MpcTracker":
            tmp = self.count_mpcstatus
            self.count_mpcstatus = 0
            if tmp == 0 :
                tmp_color = self.red
            elif str(self.mpcstatus.collision_avoidance_active) != "True":
                tmp_color = self.red
            else:
                tmp_color = self.green

            stdscr.attron(tmp_color)
            stdscr.addstr(5,  60 + self.max_length, " C.Avoid: " + str(self.mpcstatus.collision_avoidance_active))
            stdscr.addstr(6, 61 + self.max_length, str(self.mpcstatus.avoidance_active_uavs))
            tmp_color = self.red
            stdscr.attron(tmp_color)
            if str(self.mpcstatus.avoiding_collision) == "True" :
                stdscr.addstr(7, 61 + self.max_length, " ! AVOIDING COLLISION ! ")

    # #} end of mpcStatus()
            
    # #{ batteryStatus()

    def batteryStatus(self, stdscr):
        tmp = self.count_battery
        self.count_battery = 0
        if tmp == 0 and self.last_battery == "N/A ":
            bat_out = "N/A "
            tmp_color = self.red
        else:
            if tmp == 0:
                battery = self.last_battery
                self.last_battery = "N/A "
            else:
                battery = self.battery.voltage
                self.last_battery = self.battery.voltage
            if battery > 17.0:
                bat_out = battery/6
            else:
                bat_out = battery/4
            bat_out = round(bat_out, 2)
            tmp_color = self.green
            if (bat_out > 3.7):
                tmp_color = self.green
            elif (bat_out > 3.55):
                tmp_color = self.yellow
            else:
                stdscr.attron(curses.A_BLINK)
                tmp_color = self.red
        stdscr.attron(tmp_color)
        stdscr.addstr(5, 26, " Battery:  " + str(bat_out) + " ")
        stdscr.addstr(5, 42, "V ")
        stdscr.attroff(curses.A_BLINK)

    # #} end of batteryStatus()
            
    # #{ rtk()

    def rtk(self, stdscr):
        tmp = self.count_bestpos
        self.count_bestpos = 0
        if tmp == 0:
            rtk = "no msgs"
            tmp_color = self.red
        else:
            rtk = self.bestpos.position_type
            tmp_color = self.green
            if rtk == "L1_INT" or rtk == "WIDE_INT" or rtk == "NARROW_INT":
                tmp_color = self.green
            elif rtk == "L1_FLOAT" or rtk == "NARROW_FLOAT":
                tmp_color = self.yellow
            else:
                tmp_color = self.red
        stdscr.attron(tmp_color)
        stdscr.addstr(3, 60 + self.max_length, " RTK: " + str(rtk) + " ")

    # #} end of rtk()
            
    # #{ gps()

    def gps(self, stdscr):
        if "GPS" in str(self.odom) or  "Gps" in str(self.odom) or "gps" in str(self.odom) or self.has_gps:
            tmp = self.count_gpsdata
            self.count_gpsdata = 0
            if tmp == 0:
                gps = ""
            else:
                gps = (self.gpsdata.position_covariance[0] + self.gpsdata.position_covariance[4] + self.gpsdata.position_covariance[8])/3
                gps = round(gps,2)
                tmp_color = self.green
                if float(gps) < 10.0:
                    tmp_color = self.green
                elif float(gps) < 20.0:
                    tmp_color = self.yellow
                else:
                    stdscr.attron(curses.A_BLINK)
                    tmp_color = self.red
            stdscr.attron(tmp_color)
            stdscr.addstr(2, 60 + self.max_length, " GPS qual: " + str(gps) + " ")
            stdscr.attroff(curses.A_BLINK)

    # #} end of  gps()

    # #{ confTopics()

    def confTopics(self, stdscr):
        for i in range(0, len(self.param_list)):
            if(self.count_list[i] > 0):
                tmp = self.count_list[i]
                self.count_list[i] = 0
            else:
                tmp = 0

            tmp_color = self.green
            if tmp == 0:
                tmp_color = self.red
            else:
                if tmp < 0.9*self.hz_list[i] or tmp > 1.1*self.hz_list[i]:
                    tmp_color = self.yellow
                if tmp > 1.1*self.hz_list[i] and self.hz_list_modifiers[i] == "+":
                    tmp_color = self.green
                if tmp < 0.9*self.hz_list[i] and self.hz_list_modifiers[i] == "-":
                    tmp_color = self.green
            stdscr.attron(tmp_color)
            stdscr.addstr(1 + i, 46, self.spacer_string)
            stdscr.addstr(1 + i, 46, " " + str(self.name_list[i]) + ": ")
            stdscr.addstr(1 + i, 46 + self.max_length + 2 + (5 - len(str(tmp))), " " + str(tmp) + " Hz ")

    # #} confTopics()

    # #{ activeController()

    def activeController(self, stdscr):
        tmp = self.count_controller
        self.count_controller = 0
        if tmp == 0:
            controller = "NO CONTROLLER"
            tmp_color = self.red
        else:
            controller = self.controller_status.controller.rsplit('/', 1)[-1]

        if controller == "So3Controller" or controller == "MpcController":
            tmp_color = self.green
        else:
            tmp_color = self.red

        stdscr.attron(tmp_color)
        stdscr.addstr(3, 0, " " + controller + " ")

        tmp_offset = len(controller) + 1

        if controller == "So3Controller":
          tmp = self.count_gains
          self.count_gains = 0
          if tmp == 0:
              cur_gains = "N/A"
              tmp_color = self.red
          else:
              cur_gains = self.gains.data
          
          if cur_gains == "soft":
              tmp_color = self.green
          elif cur_gains == "supersoft" or cur_gains == "tight":
              tmp_color = self.yellow
          else:
              tmp_color = self.red
          
          stdscr.attroff(tmp_color)
          stdscr.addstr(3, tmp_offset, "/")
          stdscr.attron(tmp_color)
          stdscr.addstr(3, tmp_offset + 1, cur_gains + " ")

    # #} end of activeController()

    # #{ thrust()

    def thrust(self, stdscr):
        tmp = self.count_attitude_target
        self.count_attitude_target = 0
        if tmp == 0:
            thrust = "N/A"
            tmp_color = self.red
        else:
            thrust = round(self.attitude_target.thrust, 3)
            tmp_color = self.green
            if thrust > 0.7 or thrust < 0.25:
                tmp_color = self.yellow
            if thrust > 0.79:
                tmp_color = self.red
        stdscr.attron(tmp_color)
        stdscr.addstr(4, 0, " Thrust: " + str(thrust) + " ")

    # #} end of thrust()

    # #{ mass()
                
    def mass(self, stdscr):
        tmp_color = curses.color_pair(0)
        stdscr.attron(tmp_color)
        set_mass = float(self.UAV_MASS.replace(",","."))
        set_mass = round(set_mass, 2)
        stdscr.addstr(6, 40,"0 kg ")
        stdscr.addstr(6, 26," UAV_MASS: " + str(set_mass))
        est_mass = round(self.attitude_cmd.total_mass, 2)
        tmp_color = self.green
        if abs(self.attitude_cmd.mass_difference) > 2.0:
            tmp_color = self.red
            stdscr.attron(curses.A_BLINK)
        elif abs(self.attitude_cmd.mass_difference) > 1.0:
           tmp_color = self.yellow
        
        stdscr.attron(tmp_color)
       
        if self.count_attitude > 0:
            self.count_attitude = 0
            stdscr.addstr(7, 40,"0 kg ")
            stdscr.addstr(7, 26," Est mass: " + str(est_mass))
        else:
            tmp_color = self.red
            stdscr.attron(tmp_color)
            stdscr.addstr(7, 26," Est mass: N/A ")
        
        stdscr.attroff(curses.A_BLINK)
        stdscr.attroff(tmp_color)
                
    # #} end of mass()

    # #{ names()

    def names(self, stdscr):
        tmp_color = curses.color_pair(0)
        stdscr.attron(tmp_color)
        stdscr.addstr(0, 0," " + str(self.UAV_NAME) + " ")
        stdscr.addstr(0, 6," " + str(self.UAV_TYPE) + " ")
        stdscr.addstr(0, 12," " + str(self.NATO_NAME) + " ")
        stdscr.attroff(tmp_color)

    # #} end of names()

    # #{ activeTracker()

    def activeTracker(self, stdscr):
        tmp = self.count_tracker
        self.count_tracker = 0
        if tmp == 0:
            self.tracker = "NO TRACKER"
            tmp_color = self.red
        else:
            self.tracker = self.tracker_status.tracker.rsplit('/', 1)[-1]

        if self.tracker == "MpcTracker":
            tmp_color = self.green
        elif self.tracker == "LineTracker" or self.tracker == "LandoffTracker" or self.tracker == "NullTracker":
            tmp_color = self.yellow
        else:
            tmp_color = self.red

        stdscr.attron(tmp_color)
        stdscr.addstr(1, 0, " " + self.tracker + " ")
        
        tmp_offset = len(self.tracker) + 1

        tmp = self.count_constraints
        self.count_constraints = 0
        if tmp == 0:
            cur_constraints= "N/A"
            tmp_color = self.red
        else:
            cur_constraints = self.constraints.data

        if cur_constraints == "medium":
            tmp_color = self.green
        elif cur_constraints == "slow" or cur_constraints == "fast" or cur_constraints == "optflow":
            tmp_color = self.yellow
        else:
            tmp_color = self.red

        stdscr.attroff(tmp_color)
        stdscr.addstr(1, tmp_offset, "/")
        stdscr.attron(tmp_color)
        stdscr.addstr(1, tmp_offset + 1, cur_constraints + " ")
        stdscr.attroff(tmp_color)

    # #} end of activeself.tracker()

    # #{ mavrosState()

    def mavrosState(self, stdscr):
        tmp = self.count_state
        self.count_state = 0
        tmp2 = "N/A"
        if tmp == 0:
            tmp = "N/A"
        else:
            tmp = self.mavros_state.armed
            tmp2 = self.mavros_state.mode
        if(str(tmp) == "True"):
            tmp = "ARMED"
            tmp_color = self.green
        else:
            tmp = "DISARMED"
            tmp_color = self.red
        stdscr.attron(tmp_color)
        stdscr.addstr(3, 26, " State: " + str(tmp) + " ")
        if(str(tmp2) == "OFFBOARD"):
            tmp_color = self.green
        elif(str(tmp2) == "POSITION" or str(tmp2) == "MANUAL" or str(tmp2) == "ALTITUDE" ):
            tmp_color = self.yellow
        else:
            tmp_color = self.red
        stdscr.attron(tmp_color)
        stdscr.addstr(4, 26, " Mode:  " + str(tmp2) + " ")
        stdscr.attroff(tmp_color)
        
        # #} end of Mavros state

    # #{ cpuLoad()
    
    def cpuLoad(self, stdscr):
        cpu_load = 0
        tmp_color = self.green
        if process.is_alive():
            queue_lock.acquire()
            if not cpu_load_queue.empty():
                cpu_load = cpu_load_queue.get_nowait()
                while not cpu_load_queue.empty():
                    cpu_load_queue.get_nowait()
            queue_lock.release()
            if(int(cpu_load) > 89):
                tmp_color = self.red
            elif(int(cpu_load) > 74):
                tmp_color = self.yellow
        else:
            tmp_color = self.red
            cpu_load = "x"
        stdscr.attron(tmp_color)
        stdscr.addstr(1, 26, " CPU load:   ")
        stdscr.addstr(1, 35 + (4 - len(str(cpu_load))), str(cpu_load))
        stdscr.addstr(1, 39, "% ")
        stdscr.attroff(tmp_color)
    
    # #} end of cpuLoad

    def __init__(self):

        # #{ Var definitions
        
        self.tracker_status = TrackerStatus()
        self.controller_status = ControllerStatus()
        self.uav_state = UavState()
        self.mavros_state = State()
        self.attitude_target = AttitudeTarget()
        self.attitude_cmd = AttitudeCommand()
        self.gains = String()
        self.constraints = String()
        self.bestpos = Bestpos()
        self.battery = BatteryState()
        self.gpsdata = NavSatFix()
        self.bumper_status = BumperStatus()
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
        self.last_battery = 0
        self.count_mpcstatus = 0
        self.count_gpsdata = 0
        self.count_uvdar = 0
        self.bumper_repulsing = 0
        self.bumper_constraining = 0
        self.has_gps = False
        self.rospack = rospkg.RosPack()

        try:
            self.UAV_NAME =str(os.environ["UAV_NAME"])
        except:
            self.ErrorShutdown(" UAV_NAME variable is not set!!! Terminating... ", stdscr, red)
        
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
    try:
        status = Status()
    except rospy.ROSInterruptException:
        pass
str

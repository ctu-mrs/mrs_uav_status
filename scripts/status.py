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

    def OdomMainCallback(self, data):
        self.odom_main = data
        self.count_odom = self.count_odom + 1

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

    def status(self, stdscr):

        rospy.init_node('status', anonymous=True)

        # Initialize some lists
        topic_list = []
        name_list = []
        hz_list = []
        hz_list_modifiers = []
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

        param_list = rospy.get_param('~want_hz', "")
        self.SENSORS = str(self.SENSORS).replace(',', ' ') 
        sensor_list = str(self.SENSORS).split(' ')
        # needed_nodes = rospy.get_param('~needed_nodes', "")
        # for i in needed_nodes:
        #     i = str(self.UAV_NAME) + "/" + i

        if str(self.PIXGARM) == "true":
            param_list.insert(0, "mavros/distance_sensor/garmin Garmin_pix 80+")
        
        if str(self.PIXGARM) == "false" and 'garmin_down' in sensor_list:
            param_list.insert(0, "garmin/range Garmin_down 80+")

        if 'realsense_brick' in sensor_list:
            param_list.insert(0, "rs_d435/color/image_raw Realsense_Brick 25+")

        if 'bluefox_brick' in sensor_list:
            param_list.insert(0, "bluefox_brick/image_raw Bluefox_Brick 25+")

        if 'bluefox_optflow' in sensor_list:
            param_list.insert(0, "bluefox_optflow/image_raw Bluefox_Optflow 60+")
            param_list.insert(0, "optic_flow/velocity Optic_flow 60+")

        if 'rplidar' in sensor_list:
            param_list.insert(0, "rplidar/scan Rplidar 10+")

        if str(self.BLUEFOX_UV_LEFT) != "":
            param_list.insert(0, "uvdar_bluefox/left/image_raw Bluefox_UV_left 70+")

        if str(self.BLUEFOX_UV_RIGHT) != "":
            param_list.insert(0, "uvdar_bluefox/right/image_raw Bluefox_UV_right 70+")

        if str(self.ODOMETRY_TYPE) == "gps":
            param_list.insert(0, "mavros/global_position/global PX4 GPS 100")
        tmp_string = ""
        for i in param_list:
            topic_list.append(i.rsplit()[0])
            tmp = i.rsplit(' ', 1)[0]
            name_list.append(tmp.split(' ', 1)[1])
            tmp_string = i.rsplit()[-1]
            if tmp_string[-1] == "-" or tmp_string[-1] == "+":
                hz_list_modifiers.append(tmp_string[-1])
                tmp_string = tmp_string[:-1]
            else:
                hz_list_modifiers.append("0")
            hz_list.append(float(tmp_string))
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
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/odometry/odom_main", Odometry, self.OdomMainCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/state", State, self.StateCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.AttitudeTargetCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/gain_manager/current_gains", String, self.GainsCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/constraint_manager/current_constraints", String, self.ConstraintsCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/tersus/bestpos", Bestpos, self.BestposCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/battery", BatteryState, self.BatteryCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/control_manager/mpc_tracker/diagnostics", MpcTrackerDiagnostics, self.MPCstatusCallback)
        rospy.Subscriber("/" + str(self.UAV_NAME) + "/mavros/global_position/global", NavSatFix, self.GPSCallback)

        for i in range(0, len(param_list)):
            if(str(topic_list[i][0]) == "/"):
                sub_list.append(rospy.Subscriber(str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            else:
                sub_list.append(rospy.Subscriber("/" + str(self.UAV_NAME) + "/" + str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            self.count_list.append(0)

        rate = rospy.Rate(1)
        time.sleep(1)

        # for topics from config list
        max_length = 0

        # disk space 
        last_remaining = 0;
        for i in range(0, len(param_list)):
            max_length = len(max(name_list, key=len))
        spacer_string = "          "
        for i in range(0, max_length):
            spacer_string = spacer_string + " "

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
        tmp_color = curses.color_pair(0)
        process.start()
        cpu_load = 0

        while not rospy.is_shutdown():
            curses.resizeterm(30, 150)
            stdscr.clear()
            stdscr.attroff(tmp_color)
            if(not dark_mode):
                stdscr.attron(curses.A_REVERSE)
            stdscr.attron(curses.A_BOLD)

            # #{ CPU LOAD

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
            stdscr.attron(tmp_color)
            stdscr.addstr(1, 26, " CPU load:   ")
            stdscr.addstr(1, 35 + (4 - len(str(cpu_load))), str(cpu_load))
            stdscr.addstr(1, 39, "% ")

            # #} end of CPU LOAD

            # #{ Mavros state

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
                tmp_color = green
            else:
                tmp = "DISARMED"
                tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(3, 26, " State: " + str(tmp) + " ")
            if(str(tmp2) == "OFFBOARD"):
                tmp_color = green
            elif(str(tmp2) == "POSITION" or str(tmp2) == "MANUAL" or str(tmp2) == "ALTITUDE" ):
                tmp_color = yellow
            else:
                tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(4, 26, " Mode:  " + str(tmp2) + " ")
            # #} end of Mavros state

            # #{ Active Tracker
            tmp = self.count_tracker
            self.count_tracker = 0
            if tmp == 0:
                tracker = "NO TRACKER"
                tmp_color = red
            else:
                tracker = self.tracker_status.tracker.rsplit('/', 1)[-1]

            if tracker == "MpcTracker":
                tmp_color = green
            elif tracker == "LineTracker" or tracker == "LandoffTracker" or tracker == "NullTracker":
                tmp_color = yellow
            else:
                tmp_color = red

            stdscr.attron(tmp_color)
            stdscr.addstr(1, 0, " " + tracker + " ")
            
            tmp_offset = len(tracker) + 1

            tmp = self.count_constraints
            self.count_constraints = 0
            if tmp == 0:
                cur_constraints= "N/A"
                tmp_color = red
            else:
                cur_constraints = self.constraints.data

            if cur_constraints == "medium":
                tmp_color = green
            elif cur_constraints == "slow" or cur_constraints == "fast" or cur_constraints == "optflow":
                tmp_color = yellow
            else:
                tmp_color = red

            stdscr.attroff(tmp_color)
            stdscr.addstr(1, tmp_offset, "/")
            stdscr.attron(tmp_color)
            stdscr.addstr(1, tmp_offset + 1, cur_constraints + " ")
            # #} end of Active Tracker

# #{ names

            stdscr.attroff(tmp_color)
            tmp_color = curses.color_pair(0)
            stdscr.attron(tmp_color)
            stdscr.addstr(0, 0," " + str(self.UAV_NAME) + " ")
            stdscr.addstr(0, 6," " + str(self.UAV_TYPE) + " ")
            stdscr.addstr(0, 12," " + str(self.NATO_NAME) + " ")
            # #} end of names

# #{ Mass
            
            stdscr.attroff(tmp_color)
            tmp_color = curses.color_pair(0)
            stdscr.attron(tmp_color)
            set_mass = float(self.UAV_MASS.replace(",","."))
            set_mass = round(set_mass, 2)
            stdscr.addstr(6, 40,"0 kg ")
            stdscr.addstr(6, 26," UAV_MASS: " + str(set_mass))
            est_mass = round(self.attitude_cmd.total_mass, 2)
            tmp_color = green
            if abs(self.attitude_cmd.mass_difference) > 2.0:
                tmp_color = red
                stdscr.attron(curses.A_BLINK)
            elif abs(self.attitude_cmd.mass_difference) > 1.0:
               tmp_color = yellow
            
            stdscr.attron(tmp_color)
           
            if self.count_attitude > 0:
                self.count_attitude = 0
                stdscr.addstr(7, 40,"0 kg ")
                stdscr.addstr(7, 26," Est mass: " + str(est_mass))
            else:
                tmp_color = red
                stdscr.attron(tmp_color)
                stdscr.addstr(7, 26," Est mass: N/A ")
            
            stdscr.attroff(curses.A_BLINK)
            
            # #} end of Mass

            # #{ Thrust
            tmp = self.count_attitude_target
            self.count_attitude_target = 0
            if tmp == 0:
                thrust = "N/A"
                tmp_color = red
            else:
                thrust = round(self.attitude_target.thrust, 3)
                tmp_color = green
                if thrust > 0.7 or thrust < 0.25:
                    tmp_color = yellow
                if thrust > 0.79:
                    tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(4, 0, " Thrust: " + str(thrust) + " ")
            # #} end of Thrust

            # #{ Active Controller
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

            stdscr.attron(tmp_color)
            stdscr.addstr(3, 0, " " + controller + " ")

            tmp_offset = len(controller) + 1

            if controller == "So3Controller":
              tmp = self.count_gains
              self.count_gains = 0
              if tmp == 0:
                  cur_gains = "N/A"
                  tmp_color = red
              else:
                  cur_gains = self.gains.data
              
              if cur_gains == "soft":
                  tmp_color = green
              elif cur_gains == "supersoft" or cur_gains == "tight":
                  tmp_color = yellow
              else:
                  tmp_color = red
              
              stdscr.attroff(tmp_color)
              stdscr.addstr(3, tmp_offset, "/")
              stdscr.attron(tmp_color)
              stdscr.addstr(3, tmp_offset + 1, cur_gains + " ")
            # #} end of Active Controller

            # #{ Odom

            tmp = self.count_odom
            self.count_odom = 0
            odom = ""
            tmp_color = green
            if tmp == 0:
                odom = "NO ODOM"
                tmp_color = red
            else:
                odom = self.odom_main.child_frame_id
                if tmp < 0.9*100 or tmp > 1.1*100:
                    tmp_color = yellow
            stdscr.attron(tmp_color)
            stdscr.addstr(5, 0, " Odom:     Hz ")
            stdscr.addstr(5, 5 + (5 - len(str(tmp))),str(tmp) + " ")
            stdscr.addstr(6, 0, " " + str(odom) + " ")
            tmp = round(self.odom_main.pose.pose.position.x,2)
            stdscr.addstr(5, 14, "       ")
            stdscr.addstr(5, 21, " X ")
            stdscr.addstr(5, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
            tmp = round(self.odom_main.pose.pose.position.y,2)
            stdscr.addstr(6, 14, "       ")
            stdscr.addstr(6, 21, " Y ")
            stdscr.addstr(6, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
            tmp = round(self.odom_main.pose.pose.position.z,2)
            stdscr.addstr(7, 14, "       ")
            stdscr.addstr(7, 21, " Z ")
            stdscr.addstr(7, 17-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
            
            quaternion = (
                self.odom_main.pose.pose.orientation.x,
                self.odom_main.pose.pose.orientation.y,
                self.odom_main.pose.pose.orientation.z,
                self.odom_main.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            tmp = round(euler[2], 2)
            stdscr.addstr(7, 4, "       ")
            stdscr.addstr(7, 9, " yaw ")
            stdscr.addstr(7, 5 - (len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

            # #} end of Odom

            # #{ Topics from config
            for i in range(0, len(param_list)):
                if(self.count_list[i] > 0):
                    tmp = self.count_list[i]
                    self.count_list[i] = 0
                else:
                    tmp = 0

                tmp_color = green
                if tmp == 0:
                    tmp_color = red
                else:
                    if tmp < 0.9*hz_list[i] or tmp > 1.1*hz_list[i]:
                        tmp_color = yellow
                    if tmp > 1.1*hz_list[i] and hz_list_modifiers[i] == "+":
                        tmp_color = green
                    if tmp < 0.9*hz_list[i] and hz_list_modifiers[i] == "-":
                        tmp_color = green
                stdscr.attron(tmp_color)
                stdscr.addstr(1 + i, 46, spacer_string)
                stdscr.addstr(1 + i, 46, " " + str(name_list[i]) + ": ")
                stdscr.addstr(1 + i, 46 + max_length + 2 + (5 - len(str(tmp))), " " + str(tmp) + " Hz ")

            # #} end of Topics from config

            # #{ Nodes running
            
            # stdscr.attroff(tmp_color)
            # tmp_color = curses.color_pair(0)
            # stdscr.attron(tmp_color)

            # nodelist = rosnode.get_node_names()
            # iterator = 3
            # iterator2 = 0
            # tmp_max_len = 0
            # tmp_max_len_latch = 0
            # stdscr.addstr(3, 60 + max_length, " Missing nodes: ")
            
            # stdscr.attroff(tmp_color)
            # stdscr.attron(red)
            # for i in needed_nodes:
            #     if not any(str(i) in s for s in nodelist):
            #         iterator = iterator + 1
            #         if iterator == 8:
            #             iterator = 4
            #             iterator2 = iterator2 + 1
            #             tmp_max_len_latch = tmp_max_len
            #             tmp_max_len = 0
            #         stdscr.addstr(iterator, 60 + max_length + (2 + tmp_max_len_latch) * iterator2, " " + str(i) + " ")
            #         if tmp_max_len < len(str(i)):
            #             tmp_max_len = len(str(i))
            # if iterator == 3 and iterator2 == 0:
            #         stdscr.attroff(red)
            #         stdscr.attron(green)
            #         stdscr.addstr(4, 60 + max_length, " None ")
            # #} end of Nodes running

            # #{ GPS

            if "GPS" in str(odom) or  "Gps" in str(odom) or "gps" in str(odom):
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
                        stdscr.attron(curses.A_BLINK)
                        tmp_color = red
                stdscr.attron(tmp_color)
                stdscr.addstr(2, 60 + max_length, " GPS qual: " + str(gps) + " ")
                stdscr.attroff(curses.A_BLINK)
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
            stdscr.attron(tmp_color)
            stdscr.addstr(3, 60 + max_length, " RTK: " + str(rtk) + " ")
            # #} end of RTK

            # #{ Battery
            tmp = self.count_battery
            self.count_battery = 0
            if tmp == 0 and self.last_battery == "N/A ":
                bat_out = "N/A "
                tmp_color = red
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
                tmp_color = green
                if (bat_out > 3.7):
                    tmp_color = green
                elif (bat_out > 3.55):
                    tmp_color = yellow
                else:
                    stdscr.attron(curses.A_BLINK)
                    tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(5, 26, " Battery:  " + str(bat_out) + " ")
            stdscr.addstr(5, 42, "V ")
            stdscr.attroff(curses.A_BLINK)
            # #} end of Battery

            # #{ mpcstatus
            if tracker == "MpcTracker":
                tmp = self.count_mpcstatus
                self.count_mpcstatus = 0
                if tmp == 0 :
                    tmp_color = red
                elif str(self.mpcstatus.collision_avoidance_active) != "True":
                    tmp_color = red
                else:
                    tmp_color = green

                stdscr.attron(tmp_color)
                stdscr.addstr(5,  60 + max_length, " C.Avoid: " + str(self.mpcstatus.collision_avoidance_active))
                stdscr.addstr(6, 61 + max_length, str(self.mpcstatus.avoidance_active_uavs))
                tmp_color = red
                stdscr.attron(tmp_color)
                if str(self.mpcstatus.avoiding_collision) == "True" :
                    stdscr.addstr(7, 61 + max_length, " ! AVOIDING COLLISION ! ")
            # #} end of mpcstatus

# #{ Misc
            
            stdscr.attroff(green)
            try:
                stdscr.addstr(0, 46, " " + str(os.readlink(str(self.rospack.get_path('mrs_general')) +"/config/world_current.yaml")) + " ")
            except:
                stdscr.attron(red)
                stdscr.attron(curses.A_BLINK)
                stdscr.addstr(0, 46," NO ARENA DEFINED! ")
                stdscr.attroff(curses.A_BLINK)
                stdscr.attroff(red)
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
                        tmp_color = yellow
                    else:
                        tmp_color = green
                elif remaining > 10:
                    tmp_color = yellow
                else:
                    tmp_color = red
                    stdscr.attron(curses.A_BLINK)
                last_remaining = remaining 
                stdscr.attron(tmp_color)
                stdscr.addstr(1, 60 + max_length, " Disk space: " + str(output) + " ")
                stdscr.attroff(curses.A_BLINK)
            except:
                stdscr.attron(red)
                stdscr.addstr(1, 60 + max_length, " Disk space: N/A ")

            # #} end of Misc
            
            stdscr.refresh()
            rate.sleep()
            

    def __init__(self):

        # #{ Var definitions
        
        self.tracker_status = TrackerStatus()
        self.controller_status = ControllerStatus()
        self.odom_main = Odometry()
        self.mavros_state = State()
        self.attitude_target = AttitudeTarget()
        self.attitude_cmd = AttitudeCommand()
        self.gains = String()
        self.constraints = String()
        self.bestpos = Bestpos()
        self.battery = BatteryState()
        self.gpsdata = NavSatFix()
        self.count_list = []
        self.count_odom = 0
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
        self.rospack = rospkg.RosPack()
        
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



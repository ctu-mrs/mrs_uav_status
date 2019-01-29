#!/usr/bin/env python
import rospkg
import rospy
import curses
import os
import time
import multiprocessing as mp
import subprocess

from rostopic import ROSTopicHz

from mrs_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
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
            cpu_load_queue.put(100-int(output));
        queue_lock.release()

cpu_load_queue = mp.Queue()
queue_lock = mp.Lock()
process = mp.Process(target=get_cpu_load, args=(cpu_load_queue, queue_lock))

class Status:

    tracker_status = TrackerStatus()
    odom_main = Odometry()
    mavros_state = State()
    count_list = []
    count_odom = 0
    count_state = 0
    count_tracker = 0
    rospack = rospkg.RosPack()

    def MultiCallback(self, data, callback_id):
        self.count_list[callback_id] = self.count_list[callback_id] + 1

    def TrackerStatusCallback(self, data):
        self.tracker_status = data
        self.count_tracker = self.count_tracker + 1

    def OdomMainCallback(self, data):
        self.odom_main = data
        self.count_odom = self.count_odom + 1

    def StateCallback(self, data):
        self.mavros_state = data
        self.count_state = self.count_state + 1

    def ErrorShutdown(self, message, stdscr, color):
        stdscr.clear()
        stdscr.attron(color)
        stdscr.attron(curses.A_BLINK)
        stdscr.attron(curses.A_BOLD)
        stdscr.addstr(1, 0,str(message))
        stdscr.refresh()
        time.sleep(10)
        rospy.signal_shutdown('Quit') 
    
    def status(self, stdscr):

        rospy.init_node('status', anonymous=True)

        # Initialize some lists
        topic_list = []
        name_list = []
        hz_list = []
        sub_list = []
        dark_mode = True;

        # Initialize curses
        curses.initscr()
        curses.start_color()
        curses.use_default_colors()

        for i in range(0, curses.COLORS):
            curses.init_pair(i + 1, i, -1)

        # Get parameters from config file and put them in lists

        param_list = rospy.get_param('~list', "");
        for i in param_list:
            topic_list.append(i.rsplit()[0])
            tmp = i.rsplit(' ', 1)[0]
            name_list.append(tmp.split(' ', 1)[1])
            hz_list.append(float(i.rsplit()[-1]))
        dark_mode = rospy.get_param('~dark_mode', True)

        # Create ROSTopicHz and subscribers defined by the loaded config
        for i in range(0, len(param_list)):
            if(str(topic_list[i][0]) == "/"):
                sub_list.append(rospy.Subscriber(str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            else:
                sub_list.append(rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/" + str(topic_list[i]), rospy.AnyMsg, self.MultiCallback, callback_args = i))
            self.count_list.append(0)

        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/tracker_status", TrackerStatus, self.TrackerStatusCallback)
        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/odometry/odom_main", Odometry, self.OdomMainCallback)
        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/mavros/state", State, self.StateCallback)

        rate = rospy.Rate(1)
        time.sleep(1)
       
        # for topics from config list
        max_length = 0
        for i in range(0, len(param_list)):
            max_length = len(max(name_list, key=len))
        spacer_string = "          "
        for i in range(0, max_length):
            spacer_string = spacer_string + " "
        
        # blue = 47, yellow = 227, red = 197
        if(dark_mode):
            blue = curses.color_pair(34)
            green = curses.color_pair(47)
            yellow = curses.color_pair(221)
            red = curses.color_pair(197)
        else:
            blue = curses.color_pair(20)
            green = curses.color_pair(29)
            yellow = curses.color_pair(173)
            red = curses.color_pair(125)
        tmp_color = curses.color_pair(0)
        process.start()
        cpu_load = 0;

        while not rospy.is_shutdown():
            stdscr.clear()
            stdscr.attroff(tmp_color)
            if(not dark_mode):
                stdscr.attron(curses.A_REVERSE)
            stdscr.attron(curses.A_BOLD)
            try:
                stdscr.addstr(0, 0," " + str(os.environ["UAV_NAME"]) + " ")
            except:
                self.ErrorShutdown(" UAV_NAME variable is not set!!! Terminating... ", stdscr, red)
            try:
                stdscr.addstr(0, 9," UAV_MASS = " + str(os.environ["UAV_MASS"] + " kg "))
            except:
                stdscr.attron(red)
                stdscr.attron(curses.A_BLINK)
                stdscr.addstr(0, 9," UAV_MASS = NOT SET! ")
                stdscr.attroff(curses.A_BLINK)
                stdscr.attroff(red)
            try:
                stdscr.addstr(0, 31, " " + str(os.readlink(str(self.rospack.get_path('mrs_general')) +"/config/world_current.yaml")) + " ")
            except:
                stdscr.attron(red)
                stdscr.attron(curses.A_BLINK)
                stdscr.addstr(0, 9," NO ARENA DEFINED! ")
                stdscr.attroff(curses.A_BLINK)
                stdscr.attroff(red)
                
            # #{ CPU LOAD

            tmp_color = blue
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
            stdscr.addstr(2, 31, " CPU load:   ")
            stdscr.addstr(2, 40 + (4 - len(str(cpu_load))), str(cpu_load))
            stdscr.addstr(2, 44, "% ")

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
                tmp_color = blue
            else:
                tmp = "DISARMED"
                tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(4, 31, " State: " + str(tmp) + " ")
            if(str(tmp2) == "OFFBOARD"):
                tmp_color = blue
            elif(str(tmp2) == "POSITION" or str(tmp2) == "MANUAL" or str(tmp2) == "ALTITUDE" ):
                tmp_color = yellow
            else:
                tmp_color = red
            stdscr.attron(tmp_color)
            stdscr.addstr(5, 31, " Mode:  " + str(tmp2) + " ")
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
                tmp_color = blue
            elif tracker == "LineTracker" or tracker == "LandoffTracker" or tracker == "NullTracker":
                tmp_color = yellow
            else:
                tmp_color = red

            stdscr.attron(tmp_color)
            stdscr.addstr(1, 0, " Active tracker: " + tracker + " ")
            # #} end of

            # #{ Odom

            tmp = self.count_odom
            self.count_odom = 0
            odom = ""
            tmp_color = blue
            if tmp == 0:
                odom = "NO ODOM"
                tmp_color = red
            else:
                odom = self.odom_main.child_frame_id
                if tmp < 0.9*100 or tmp > 1.1*100:
                    tmp_color = yellow
            stdscr.attron(tmp_color)
            stdscr.addstr(3, 0, " Odom:     Hz, " + str(odom))
            stdscr.addstr(3, 5 + (5 - len(str(tmp))),str(tmp) + " ")
            tmp = round(self.odom_main.pose.pose.position.x,2)
            stdscr.addstr(4, 2, "       ")
            stdscr.addstr(4, 9, " X ")
            stdscr.addstr(4, 5-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
            tmp = round(self.odom_main.pose.pose.position.y,2)
            stdscr.addstr(5, 2, "       ")
            stdscr.addstr(5, 9, " Y ")
            stdscr.addstr(5, 5-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")
            tmp = round(self.odom_main.pose.pose.position.z,2)
            stdscr.addstr(6, 2, "       ")
            stdscr.addstr(6, 9, " Z ")
            stdscr.addstr(6, 5-(len(str(tmp).split('.')[0])), " " + str(tmp) + " ")

            # #} end of Odom

            # #{ Topics from config
            for i in range(0, len(param_list)):
                if(self.count_list[i] > 0):
                    tmp = self.count_list[i]
                    self.count_list[i] = 0
                else:
                    tmp = 0

                tmp_color = blue
                if tmp == 0:
                    tmp_color = red
                else:
                    if tmp < 0.9*hz_list[i] or tmp > 1.1*hz_list[i]:
                        tmp_color = yellow

                stdscr.attron(tmp_color)
                stdscr.addstr(1 + i, 50, spacer_string)
                stdscr.addstr(1 + i, 50, " " + str(name_list[i]) + ": ")
                stdscr.addstr(1 + i, 50 + max_length + 2 + (5 - len(str(tmp))), " " + str(tmp) + " Hz ")

            # #} end of Topics from config

            stdscr.refresh()
            rate.sleep()

    def __init__(self):
        curses.wrapper(self.status)

if __name__ == '__main__':
    try:
        status = Status()
    except rospy.ROSInterruptException:
        pass

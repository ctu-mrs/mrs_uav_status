#!/usr/bin/env python

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

    def status(self, stdscr):

        rospy.init_node('status', anonymous=True)

        # Initialize some lists
        topic_list = []
        name_list = []
        hz_list = []
        sub_list = []

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

        # green = 47, yellow = 227, red = 197
        green = curses.color_pair(47)
        yellow = curses.color_pair(221)
        red = curses.color_pair(197)
        tmp_color = curses.color_pair(0)
        process.start()
        cpu_load = 0;
        while not rospy.is_shutdown():
            stdscr.clear()
            stdscr.addstr(0, 36,str(os.environ["UAV_NAME"]), curses.A_BOLD)

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
            stdscr.addstr(2, 31, "CPU load: ", tmp_color)
            stdscr.addstr(2, 39 + (4 - len(str(cpu_load))), str(cpu_load), tmp_color)
            stdscr.addstr(2, 44, "%", tmp_color)

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
            stdscr.addstr(4, 31, "State: " + str(tmp), tmp_color)
            if(str(tmp2) == "OFFBOARD"):
                tmp_color = green
            elif(str(tmp2) == "POSITION" or str(tmp2) == "MANUAL" or str(tmp2) == "ALTITUDE" ):
                tmp_color = yellow
            else:
                tmp_color = red
            stdscr.addstr(5, 31, "Mode:  " + str(tmp2), tmp_color)
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

            stdscr.addstr(1, 0, "Active tracker: " + tracker, tmp_color)
            # #} end of

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
            stdscr.addstr(3, 0, "Odom:     Hz, " + str(odom), tmp_color)
            stdscr.addstr(3, 4+ (5 - len(str(tmp))),str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.x,2)
            stdscr.addstr(4, 10, "X", tmp_color)
            stdscr.addstr(4, 6-(len(str(tmp).split('.')[0])), str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.y,2)
            stdscr.addstr(5, 10, "Y", tmp_color)
            stdscr.addstr(5, 6-(len(str(tmp).split('.')[0])), str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.z,2)
            stdscr.addstr(6, 10, "Z", tmp_color)
            stdscr.addstr(6, 6-(len(str(tmp).split('.')[0])), str(tmp), tmp_color)

            # #} end of Odom

            # #{ Topics from config
            max_length = 0
            for i in range(0, len(param_list)):
                max_length = len(max(name_list, key=len))
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

                stdscr.addstr(1 + i, 50, str(name_list[i]) + ": ", tmp_color)
                stdscr.addstr(1 + i, 50 + max_length + 2 + (5 - len(str(tmp))), str(tmp) + " Hz", tmp_color)

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
#!/usr/bin/env python

import rospy
import curses
import os
import time

from rostopic import ROSTopicHz 

from mrs_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

class Status:
    tracker_status = TrackerStatus()
    odom_main = Odometry()

    def TrackerStatusCallback(self, data):
        tracker_status = data

    def OdomMainCallback(self, data):
        odom_main = data
        

    # def AttitudeCommandCallback(self, data):
    #     self.attitude_command_counter += 1

    # def GenericCallback(self, data, args):
    #     print("called")

    def status(self, stdscr):
      
        rospy.init_node('status', anonymous=True)
        
        # Initialize some lists
        topic_list = []
        name_list = []
        hz_list = []
        sub_list = []
        rt_list = []
       
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
            name_list.append(i.rsplit()[1])
            hz_list.append(float(i.rsplit()[2]))

        # Create ROSTopicHz and subscribers defined by the loaded config
        for i in range(0, len(param_list)):
            rt_list.append(ROSTopicHz(hz_list[i]))
            sub_list.append(rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/" + str(topic_list[i]), rospy.AnyMsg, rt_list[i].callback_hz))


        # rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/tracker_status", TrackerStatus, self.TrackerStatusCallback)
        # rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/attitude_cmd", AttitudeCommand, self.AttitudeCommandCallback)
        
        rt_tracker_status = ROSTopicHz(10)
        rt_attitude_cmd = ROSTopicHz(100)
        rt_odom_main = ROSTopicHz(100)

        rt_sub_tracker_status = rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/tracker_status", rospy.AnyMsg, rt_tracker_status.callback_hz)        
        rt_sub_attitude_cmd = rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/attitude_cmd", rospy.AnyMsg, rt_attitude_cmd.callback_hz)        
        rt_sub_odom_main = rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/odometry/odom_main", rospy.AnyMsg, rt_odom_main.callback_hz)        

        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/tracker_status", TrackerStatus, self.TrackerStatusCallback)
        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/odometry/odom_main", Odometry, self.OdomMainCallback)

        rate = rospy.Rate(10)
        time.sleep(0.1)

        attitude_command_hz = 0;
        tracker_status_hz = 0;
        
        # green = 47, yellow = 227, red = 197
        green = curses.color_pair(47)
        yellow = curses.color_pair(227)
        red = curses.color_pair(197)
        tmp_color = curses.color_pair(0)

        while not rospy.is_shutdown():
            stdscr.clear()
            stdscr.addstr(0, 0, str(os.environ["UAV_NAME"]))
            # stdscr.addstr(1, 0, "Active tracker: " + self.tracker_status)
            # tracker = self.tracker_status.tracker.rsplit('/', 1)[-1]

            if tracker_status_hz == 0:
                tracker = "NO TRACKER"

            if tracker == "MpcTracker":
                tmp_color = green
            elif tracker == "LineTracker" or tracker == "LandoffTracker" or tracker == "NullTracker":
                tmp_color = yellow
            else:
                tmp_color = red


            for i in range(0, len(param_list)):
                
                tmp = rt_list[i].get_hz()
                tmp_color = green

                if tmp == None:
                    tmp = "no messages"
                    tmp_color = red
                else:
                    tmp = round(tmp[0],1)
                    if tmp < 0.9*hz_list[i] or tmp > 1.1*hz_list[i]:
                        tmp_color = yellow

                stdscr.addstr(8 + i, 0, str(name_list[i]) + ": " + str(tmp), tmp_color)




            stdscr.addstr(1, 16, tracker, tmp_color)
            stdscr.addstr(6, 16, str(len(param_list)), tmp_color)
            stdscr.addstr(2, 0, "Attitude cmd rate: ")
            # tmp = rt.get_hz()
            # if tmp == None:
            #     stdscr.addstr(2, 20,"0 hz", red)
            # else:
            #     stdscr.addstr(2, 20, str(round(tmp[0],1)) + " hz", green)
            stdscr.refresh()
            rate.sleep()

    def __init__(self):
        curses.wrapper(self.status)

if __name__ == '__main__':
    try:
        status = Status()
    except rospy.ROSInterruptException:
        pass




    # stdscr.addstr(0, 0, os.environ["UAV_NAME"])
    # stdscr.attron(curses.A_BLINK)
    # stdscr.addstr(3, 0, "Hello " + str(i), curses.color_pair(197) | curses.A_BLINK)
    # stdscr.attroff(curses.A_BLINK)

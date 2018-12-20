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
        self.tracker_status = data

    def OdomMainCallback(self, data):
        self.odom_main = data

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

        
        rt_tracker_status = ROSTopicHz(10)
        rt_attitude_cmd = ROSTopicHz(100)
        rt_odom_main = ROSTopicHz(100)


        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/tracker_status", TrackerStatus, self.TrackerStatusCallback)
        rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/odometry/odom_main", Odometry, self.OdomMainCallback)

        rt_sub_tracker_status = rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/tracker_status", rospy.AnyMsg, rt_tracker_status.callback_hz)        
        rt_sub_attitude_cmd = rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/control_manager/attitude_cmd", rospy.AnyMsg, rt_attitude_cmd.callback_hz)        
        rt_sub_odom_main = rospy.Subscriber("/" + str(os.environ["UAV_NAME"]) + "/odometry/odom_main", rospy.AnyMsg, rt_odom_main.callback_hz)        
        
        rate = rospy.Rate(10)
        time.sleep(0.1)

        # green = 47, yellow = 227, red = 197
        green = curses.color_pair(47)
        yellow = curses.color_pair(227)
        red = curses.color_pair(197)
        tmp_color = curses.color_pair(0)

        while not rospy.is_shutdown():
            stdscr.clear()
            stdscr.addstr(0, 0, str(os.environ["UAV_NAME"]))

            tmp_hz = rt_tracker_status.get_hz()

            if tmp_hz > 0.0:
                tracker = self.tracker_status.tracker.rsplit('/', 1)[-1]
            else:
                tracker = "NO TRACKER"

            if tracker == "MpcTracker":
                tmp_color = green
            elif tracker == "LineTracker" or tracker == "LandoffTracker" or tracker == "NullTracker":
                tmp_color = yellow
            else:
                tmp_color = red
            
            stdscr.addstr(2, 0, "Active tracker: " + tracker, tmp_color)
            
            tmp = rt_attitude_cmd.get_hz()
            tmp_color = green
            if tmp == None:
                tmp = "0.0"
                tmp_color = red
            else:
                tmp = round(tmp[0],1)
                if tmp < 0.9*100 or tmp > 1.1*100:
                    tmp_color = yellow
            stdscr.addstr(3, 0, "Attitude cmd rate: ", tmp_color)
            stdscr.addstr(3, 20 + (4 - len(str(tmp))),str(tmp) + " Hz", tmp_color)

            tmp = rt_odom_main.get_hz()
            odom = ""
            tmp_color = green
            if tmp == None:
                tmp = "0.0"
                odom = "NO ODOM"
                tmp_color = red
            else:
                odom = self.odom_main.child_frame_id
                tmp = round(tmp[0],1)
                if tmp < 0.9*100 or tmp > 1.1*100:
                    tmp_color = yellow
            stdscr.addstr(5, 0, "Odom:      Hz, " + str(odom), tmp_color)
            stdscr.addstr(5, 6+ (5 - len(str(tmp))),str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.x,1)
            stdscr.addstr(6, 0, "X: " + str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.y,1)
            stdscr.addstr(7, 0, "Y: " + str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.z,1)
            stdscr.addstr(8, 0, "Z: " + str(tmp), tmp_color)









            
            max_length = 0
            for i in range(0, len(param_list)):
                max_length = len(max(name_list, key=len))
            for i in range(0, len(param_list)):
                
                tmp = rt_list[i].get_hz()
                tmp_color = green

                if tmp == None:
                    tmp = "0.0"
                    tmp_color = red
                else:
                    tmp = round(tmp[0],1)
                    if tmp < 0.9*hz_list[i] or tmp > 1.1*hz_list[i]:
                        tmp_color = yellow

                stdscr.addstr(11 + i, 0, str(name_list[i]) + ": ", tmp_color)
                stdscr.addstr(11 + i, max_length + 2 + (5 - len(str(tmp))), str(tmp) + " Hz", tmp_color)

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

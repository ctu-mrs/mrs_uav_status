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

def get_cpu_load(cpu_load_queue, queue_lock):
    run = True
    while run:
        bashCommand = "vmstat 1 2"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        output = output.split()[-3]
        queue_lock.acquire()
        if not cpu_load_queue.empty():
            run = False
        else:
            cpu_load_queue.put(100-int(output));
        queue_lock.release()

cpu_load_queue = mp.Queue() 
queue_lock = mp.Lock()

class Status:
    tracker_status = TrackerStatus()
    odom_main = Odometry()
    process = mp.Process(target=get_cpu_load, args=(cpu_load_queue, queue_lock))
    
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
            tmp = i.rsplit(' ', 1)[0]
            name_list.append(tmp.split(' ', 1)[1])
            hz_list.append(float(i.rsplit()[-1]))

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
        yellow = curses.color_pair(221)
        red = curses.color_pair(197)
        tmp_color = curses.color_pair(0)
# , args=(x, source_ys, source_x, target_ys[x], target_x, foils, timepix_segment, direct_rays_queue, reflected_ray_queue, stdout_lock, queue_lock, max_reflections, critical_angle))
        self.process.start()
        cpu_load = 0;

        while not rospy.is_shutdown():
            stdscr.clear()
            now = rospy.get_rostime().to_sec() - 1.0
            stdscr.addstr(0, 0,str(os.environ["UAV_NAME"]), curses.A_BOLD)

            if self.process.is_alive():
                queue_lock.acquire()
                if not cpu_load_queue.empty():
                    cpu_load = cpu_load_queue.get_nowait()
                queue_lock.release()
            tmp_color = green
            if(int(cpu_load) > 89):
                tmp_color = red
            elif(int(cpu_load) > 74):
                tmp_color = yellow
            stdscr.addstr(0, 15, "CPU load: " + str(cpu_load), tmp_color)
            
            tmp_tn = rt_tracker_status.get_last_printed_tn()
            tmp = rt_tracker_status.get_hz()
            if (rt_tracker_status.get_msg_tn() < now):
                tmp = None
            rt_tracker_status.set_last_printed_tn(tmp_tn)
            if tmp == None:
                tmp = "0.0"
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
            
            tmp_tn = rt_attitude_cmd.get_last_printed_tn()
            tmp = rt_attitude_cmd.get_hz()
            if (rt_attitude_cmd.get_msg_tn() < now):
                tmp = None
            rt_attitude_cmd.set_last_printed_tn(tmp_tn)
            tmp_color = green
            if tmp == None:
                tmp = "0.0"
                tmp_color = red
            else:
                tmp = round(tmp[0],1)
                if tmp < 0.9*100 or tmp > 1.1*100:
                    tmp_color = yellow
            stdscr.addstr(2, 0, "Attitude cmd rate: ", tmp_color)
            stdscr.addstr(2, 20 + (4 - len(str(tmp))),str(tmp) + " Hz", tmp_color)

            tmp_tn = rt_odom_main.get_last_printed_tn()
            tmp = rt_odom_main.get_hz()
            if (rt_odom_main.get_last_printed_tn() < now):
                tmp = None
            rt_odom_main.set_last_printed_tn(tmp_tn)
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
            stdscr.addstr(4, 0, "Odom:        Hz, " + str(odom), tmp_color)
            stdscr.addstr(4, 7+ (5 - len(str(tmp))),str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.x,2)
            stdscr.addstr(5, 10, "X", tmp_color)
            stdscr.addstr(5, 6-(len(str(tmp).split('.')[0])), str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.y,2)
            stdscr.addstr(6, 10, "Y", tmp_color)
            stdscr.addstr(6, 6-(len(str(tmp).split('.')[0])), str(tmp), tmp_color)
            tmp = round(self.odom_main.pose.pose.position.z,2)
            stdscr.addstr(7, 10, "Z", tmp_color)
            stdscr.addstr(7, 6-(len(str(tmp).split('.')[0])), str(tmp), tmp_color)
            
            max_length = 0
            for i in range(0, len(param_list)):
                max_length = len(max(name_list, key=len))
            for i in range(0, len(param_list)):
                tmp_tn = rt_list[i].get_last_printed_tn()
                tmp = rt_list[i].get_hz()
                if (rt_list[i].get_msg_tn() < (now)):
                    tmp = None
                rt_list[i].set_last_printed_tn(tmp_tn)
                tmp_color = green

                if tmp == None:
                    tmp = "0.0"
                    tmp_color = red
                else:
                    tmp = round(tmp[0],1)
                    if tmp < 0.9*hz_list[i] or tmp > 1.1*hz_list[i]:
                        tmp_color = yellow

                stdscr.addstr(1 + i, 35, str(name_list[i]) + ": ", tmp_color)
                stdscr.addstr(1 + i, 35 + max_length + 2 + (5 - len(str(tmp))), str(tmp) + " Hz", tmp_color)
            stdscr.refresh()
            rate.sleep()

    def __init__(self):
        curses.wrapper(self.status)

    def __del__(self):
        cpu_load_queue.put("stop");
        self.process.terminate()

if __name__ == '__main__':
    try:
        status = Status()
    except rospy.ROSInterruptException:
        pass

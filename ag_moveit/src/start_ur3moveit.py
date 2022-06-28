#!/usr/bin/env  python
# -*- coding: utf-8 -*-
import rospy
# from Tkinter import *
# import tkMessageBox
#import Tkinter
import os
import sys
import threading
import subprocess
import commands
import time
import signal

def main():
    # rospy.init_node('start_ur3_moveit', anonymous = True, disable_signals = True)
    launch_handle = subprocess.Popen(["roslaunch ur_bringup ur3_bringup.launch"],shell=True)
    print ("roslaunch ur_bringup ur3_bringup.launch:{}".format(launch_handle))
    time.sleep(1)
    launch_handle = subprocess.Popen(["roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch"],shell=True)
    print("roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch:{}".format(launch_handle))
    while not rospy.is_shutdown():
        rospy.sleep(2)
    launch_handle = subprocess.Popen(["kill -9 $(ps aux | grep ur3_bringup.launch | grep -v grep | awk '{print $2}')"],shell=True)
    print("kill roslaunch ur_bringup ur3_bringup.launch:{}".format(launch_handle))
    launch_handle = subprocess.Popen(["kill -9 $(ps aux | grep ur3_moveit_planning_execution.launch | grep -v grep | awk '{print $2}')"],shell=True)
    print("kill roslaunch ur_bringup ur3_bringup.launch:{}".format(launch_handle))
    launch_handle = subprocess.Popen(["kill -9 $(ps aux | grep roslaunch | grep -v grep | awk '{print $2}')"],shell=True)
    os._exit(0)

def exit_handle(signum, frame):
	print("You choose to stop me through ctrl+c.")
	sys.exit()

if __name__ == '__main__':
	# register exit function for exit by 'ctrl+c'
	signal.signal(signal.SIGINT, exit_handle)
	signal.signal(signal.SIGTERM, exit_handle)
	main()
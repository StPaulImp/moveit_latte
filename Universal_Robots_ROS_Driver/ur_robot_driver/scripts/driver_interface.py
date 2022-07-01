#!/usr/bin/env python
# coding=utf-8
# from install.lib.ur_robot_driver.scripts.lubancmd.luban_cmd import LubanCmd
# import roslib; roslib.load_manifest('ur_robot_driver')#,roslib.load_manifest('robotiq_2f_gripper_control')#,roslib.load_manifest('ag_urarm')
import time, sys, threading, math, os
import copy
import datetime
import socket, select
import struct
import traceback, code
import optparse
import SocketServer
import logging
import json
# import time as clock
# import os

import rospy
import actionlib
from std_msgs.msg import String,Int32
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction,GripperCommandAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import WrenchStamped
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_commander import MoveGroupCommander, MoveItCommanderException
from moveit_ros_planning_interface import _moveit_robot_interface
from moveit_msgs.msg import RobotState
from visualization_msgs.msg import MarkerArray
import moveit_commander.conversions as conversions

# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

# from dynamic_reconfigure.server import Server
# from ur_driver.cfg import URDriverConfig
#
# from ur_driver.deserialize import RobotState, RobotMode
# from ur_driver.deserializeRT import RobotStateRT
#
# from ur_msgs.srv import SetPayload, SetIO
# from ur_msgs.msg import *

# abs_file = os.path.abspath(os.path.dirname("__file__"))
abs_file = os.path.abspath(os.path.dirname(__file__))
print("abs_file:",abs_file)
sys.path.append(abs_file + "/lubancmd")
print("sys.path:",sys.path)
from luban_cmd import LubanCmd, ArmCtrl #ArmInterface #, GripperInterface, PoseData

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "LGPLv3"

PORT=30002       # 10 Hz, RobotState
RT_PORT=30003    #125 Hz, RobotStateRT
DEFAULT_REVERSE_PORT = 50001     #125 Hz, custom data (from prog)

#Max Velocity accepted by ur_driver
MAX_VELOCITY = 1.0
#Using a very high value in order to not limit execution of trajectories being sent from MoveIt!

#Bounds for SetPayload service
MIN_PAYLOAD = 0.0
MAX_PAYLOAD = 10.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class RobotException(Exception):
    pass


def log(s):
    print ("[%s] %s" % (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), s))

pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
pub_wrench = rospy.Publisher('wrench', WrenchStamped, queue_size=1)

class URTrajectoryFollower(object):
    """
    Python interface to socket interface of UR robot.
    programs are send to port 3002
    data is read from secondary interface(10Hz?) and real-time interface(125Hz) (called Matlab interface in documentation)
    Since parsing the RT interface uses som CPU, and does not support all robots versions, it is disabled by default
    The RT interfaces is only used for the get_force related methods
    Rmq: A program sent to the robot i executed immendiatly and any running program is stopped
    """
    RATE = 0.01
    FEEDBACK_RATE = RATE
    TIMEOUT = 1.0

    DISCONNECTED = 0
    CONNECTED = 1
    READY_TO_PROGRAM = 2
    EXECUTING = 3

    def __init__(self, proxy_ip, adapter_name, goal_time_tolerance=None, logger=False, use_robotiq = False):
        # self._log = logger
        self.adapter_name = adapter_name
        self.csys = None
        self.following_lock = threading.Lock()
        self._LubanCmd = LubanCmd(host_ip=proxy_ip)
        self._ArmCtrl = None

        # self.gripper_interface_1 = None
        # self.command_gripper = None
        # self.status = inputMsg.Robotiq2FGripper_robot_input()
        # self.gripper_statu_update = False
        # self.state_deadline_1 = None
        # self.state_deadline_2 = None
        try:
            self._ArmCtrl = ArmCtrl(arm_name = adapter_name, host_ip = proxy_ip)
            if use_robotiq == True:
                # self.gripper_interface_1 = GripperInterface(proxy_ip= proxy_ip, adapter_name = adapter_name, actuator_name = gripper_actuator_name,  client_name = gripper_client_name)
                rospy.sleep(1)
        except:
            #throw_exception_to_server(self.exception_pub, module_name = __file__, arm = 'armx', exception = coffeeStats.ARM_LEFT_NET.value, reason='UR3 connection fail. when init. ')
            raise("arm init error")

        self.goal_time_tolerance = goal_time_tolerance or rospy.Duration(0.0)
        self.joint_goal_tolerances = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.T0 = time.time()
        self.robot = None
        self.arm_client = None

        self.__keep_running = True
        self.__thread_client = threading.Thread(name = "arm_client", target = self.startclient)
        self.__thread_client.daemon = True
        self.__thread_client.start()
        rospy.sleep(2.0)
        # self.adapter_name
        self.server = actionlib.ActionServer(self.adapter_name + "/scaled_pos_joint_traj_controller/follow_joint_trajectory",
                                             FollowJointTrajectoryAction,
                                             self.on_goal, self.on_cancel, auto_start=False)

        self.goal_handle = None
        self.traj = None
        self.traj_t0 = 0.0
        self.traj_t1 = 0
        self.traj_t2 = 0
        self.first_waypoint_id = 10
        self.tracking_i = 0
        self.pending_i = 0
        self.last_point_sent = True
        #update robot state
        self.update_timer = rospy.Timer(rospy.Duration(self.FEEDBACK_RATE), self._update)
        # self.__thread_state = threading.Thread(name = "_state_update", target = self._update)
        # self.__thread_state.daemon = True
        # self.__thread_state.start()
        self.last_joint_states = None
        self.joint_lock = threading.Lock()
        #send moveit command
        self.__thread_cmd = threading.Thread(name = "_cmd_control", target = self._cmd_control)#, args = (self.arm_client,)
        self.__thread_cmd.daemon = True
        self.__thread_cmd.start()

        # self.pub_gripper_state = rospy.Publisher(self.adapter_name+'_gripper/follow_joint_trajectory/input', inputMsg.Robotiq2FGripper_robot_input, queue_size=10)
        # rospy.Subscriber(self.adapter_name+'_gripper/follow_joint_trajectory/output', outputMsg.Robotiq2FGripper_robot_output, self.refreshCommand)

    def loghandle(self, level, msg):
        if self._log:
            handler = getattr(self._log, level)
            handler("[%s] %s" % (self.__class__.__name__, msg))

    def close(self):
        self._ArmCtrl.rt_play_end()
        self.__thread_client.join()
        del self.__thread_client

    def __del__(self):
        self.close()
        self.loghandle("info","Closing servojs to robot")

    def startclient(self):
        # self.adapter_name
        print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        arm_client = actionlib.SimpleActionClient(self.adapter_name + '/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print ("Waiting for {}/scaled_pos_joint_traj_controller/follow_joint_trajectory server...".format(self.adapter_name))
        arm_client.wait_for_server()
        print ("Connected to {}/scaled_pos_joint_traj_controller/follow_joint_trajectory server".format(self.adapter_name))

    #driver
    def set_robot(self, robot):
        # Cancels any goals in progress
        if self.goal_handle:
            self.goal_handle.set_canceled()
            self.goal_handle = None
        self.traj = None
        self.robot = robot
        if self.robot:
            self.init_traj_from_robot()

    # Sets the trajectory to remain stationary at the current position
    # of the robot.
    def init_traj_from_robot(self):
        if not self.robot:
            # with self.following_lock:
            #     reconnect_robot()
            raise Exception("No robot connected")
            return
        self.traj_t0 = time.time()
        print ("init_traj_from_robot:",self.traj_t0)

        # Busy wait (avoids another mutex)
        state = self.get_joint_states()
        while not state:
            state = self.get_joint_states()
            time.sleep(0.1)

        self.traj = JointTrajectory()
        self.traj.joint_names = joint_names
        self.traj.points = [JointTrajectoryPoint(
        positions = state.position,
        velocities = [0.0] * 6,
        accelerations = [0.0] * 6,
        time_from_start = rospy.Duration(0.0))]

        print ("self.traj.points[-1]\n",self.traj.points[-1])
        print ("self.traj.points[0]\n",self.traj.points[0])


    def start(self):
        self.server.start()
        print ("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        log("on_goal ++++++++++++++++++++++++++++++++++++++")
        
        # Checks that the robot is connected
        if not self.robot:
            rospy.logerr("Received a goal, but the robot is not connected")
            goal_handle.set_rejected()
            return
        
        # Checks if the joints are just incorrect
        # print "trajectory.joint_names",goal_handle.get_goal().trajectory.joint_names
        arm_joint_names = joint_names[:6]
        if set(goal_handle.get_goal().trajectory.joint_names) != set(arm_joint_names):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" % \
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        if not self.traj_is_finite(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal with infinites or NaNs")
            goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
            return

        # Checks that the trajectory has velocities
        if not self.has_velocities(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal without velocities")
            goal_handle.set_rejected(text="Received a goal without velocities")
            return

        # Checks that the velocities are withing the specified limits
        if not self.has_limited_velocities(goal_handle.get_goal().trajectory):
            message = "Received a goal with velocities that are higher than %f" % max_velocity
            rospy.logerr(message)
            goal_handle.set_rejected(text=message)
            return

        stamp = rospy.get_rostime()
        stateRTJ = self.get_joint_states()
        if stateRTJ is None or abs(stamp - stateRTJ.header.stamp).to_sec() > 0.2:
            message = "joint state didn't updated over 200ms"
            rospy.logerr(message)
            goal_handle.set_rejected(text=message)
            return
        elif self.traj.points[-1].positions[0] != 0 or self.traj.points[-1].positions[1] != 0 or self.traj.points[-1].positions[2] != 0 or self.traj.points[-1].positions[3] != 0 or self.traj.points[-1].positions[4] != 0 or self.traj.points[-1].positions[5] != 0:
            self.traj = JointTrajectory()
            self.traj.joint_names = joint_names
            self.traj.points = [JointTrajectoryPoint(
            positions = stateRTJ.position,
            velocities = stateRTJ.velocity,
            accelerations = [0]*6,
            time_from_start = rospy.Duration(0.0))]
            print ("self.traj.points[-1]\n",self.traj.points[-1])
            print ("self.traj.points[0]\n",self.traj.points[0])

        # Orders the joints of the trajectory according to joint_names
        self.reorder_traj_joints(goal_handle.get_goal().trajectory, arm_joint_names)

        with self.following_lock:
            if self.goal_handle:
                # Cancels the existing goal
                self.goal_handle.set_canceled()
                self.first_waypoint_id += len(self.goal_handle.get_goal().trajectory.points)
                self.goal_handle = None
           
            # Inserts the current setpoint at the head of the trajectory
            now = time.time()
            point0 = self.sample_traj(self.traj, now - self.traj_t0)
            print ("point0",point0)
            point0.time_from_start = rospy.Duration(0.0)
            goal_handle.get_goal().trajectory.points.insert(0, point0)
            self.traj_t0 = now
            self.traj_t1 = now
            self.traj_t2 = now
            # Replaces the goal
            self.goal_handle = goal_handle
            self.traj = goal_handle.get_goal().trajectory
            self.goal_handle.set_accepted()
            self.last_point_sent = False
            self._ArmCtrl.rt_play_start(self.RATE*1000)
   


    def on_cancel(self, goal_handle):
        log("on_cancel")
        if goal_handle == self.goal_handle:
            with self.following_lock:
                # Uses the next little bit of trajectory to slow to a stop
                STOP_DURATION = 0.2
                now = time.time()
                point0 = self.sample_traj(self.traj, now - self.traj_t0)
                point0.time_from_start = rospy.Duration(0.0)
                point1 = self.sample_traj(self.traj, now - self.traj_t0 + STOP_DURATION)
                point1.velocities = [0.0] * 6
                point1.accelerations = [0.0] * 6
                point1.time_from_start = rospy.Duration(STOP_DURATION)
                self.traj_t0 = now
                self.traj_t1 = now
                self.traj_t2 = now
                self.traj = JointTrajectory()
                self.traj.joint_names = joint_names
                self.traj.points = [point0, point1]
                self._ArmCtrl.rt_play_end()
                self.goal_handle.set_canceled()
                self.goal_handle = None
        else:
            goal_handle.set_canceled()

    def _cmd_control(self):
        while not rospy.is_shutdown():
            # print "_cmd_control",time.time()
            rospy.sleep(self.RATE)
            if self.robot and self.traj:
                now = time.time()
                self.traj_t1 = now
                if not self.last_point_sent:
                    if (now - self.traj_t0) <= self.traj.points[-1].time_from_start.to_sec():
                        self.last_point_sent = False
                        #sending intermediate points
                        setpoint = self.sample_traj(self.traj, now - self.traj_t0)
                        try:
                            self._ArmCtrl.rt_play_data(setpoint.positions)
                        except socket.error:
                            print ("send socket.error",socket.error)
                    else:
                        # All intermediate points sent, sending last point to make sure we
                        # reach the goal.
                        # This should solve an issue where the robot does not reach the final
                        # position and errors out due to not reaching the goal point.
                        last_point = self.traj.points[-1]
                        state = self.get_joint_states()
                        if state is None:
                            continue
                        position_in_tol = self.within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                        # Performing this check to try and catch our error condition.  We will always
                        # send the last point just in case.
                        if not position_in_tol:
                            rospy.logwarn("Trajectory time exceeded and current robot state not at goal, last point required")
                            rospy.logwarn("Current trajectory time: %s, last point time: %s" % \
                                        (now - self.traj_t0, self.traj.points[-1].time_from_start.to_sec()))
                            rospy.logwarn("Desired: %s\nactual: %s\n%s\nvelocity: %s\n" % \
                                        (last_point.positions, state.position, self.rad2deg(state.position),state.velocity))
                        setpoint = self.sample_traj(self.traj, self.traj.points[-1].time_from_start.to_sec())
                        print ("setpoint 2",setpoint)
                        try:
                            print ("+++++++++++ if not self.last_point_sent: \n if not position_in_tol: \n" ,setpoint.positions)
                            self._ArmCtrl.rt_play_data(setpoint.positions)
                            self.last_point_sent = True
                        except socket.error:
                            raise ("last_point_sent has socket error",socket.error)
                else:  # Off the end
                    if self.goal_handle:
                        last_point = self.traj.points[-1]
                        state = self.get_joint_states()
                        if state is None:
                            continue
                        position_in_tol = self.within_tolerance(state.position[:5], last_point.positions, [0.05]*6)
                        velocity_in_tol = self.within_tolerance(state.velocity, last_point.velocities, [0.05]*6)

                        if now - (self.traj_t0 + last_point.time_from_start.to_sec()) > self.goal_time_tolerance:
                            # Took too long to reach the goal.  Aborting
                            rospy.logwarn("Took too long to reach the goal.\nDesired: %s\nactual: %s\n%s\nvelocity: %s" % \
                                             (last_point.positions, state.position, self.rad2deg(state.position), state.velocity))
                            self._ArmCtrl.rt_play_end()
                            self.goal_handle.set_aborted(text="Took too long to reach the goal")
                            self.goal_handle = None
                            print("+++++ goal set_aborted!")
                        # if position_in_tol and velocity_in_tol:
                        else:
                            self._ArmCtrl.rt_play_end()
                            # The arm reached the goal (and isn't moving).  Succeeding
                            self.goal_handle.set_succeeded()
                            self.goal_handle = None
                            print("+++++ goal succeeded!")
                            
    def _update(self,event):
        #200ms tolerence
        poseinfo_tolerence_period = 0.2
        if not rospy.is_shutdown() and self._ArmCtrl:
            # print "_update",time.time()
            # rospy.sleep(self.RATE)
            [ret, joint, timestamp] = self._ArmCtrl.get_joints(with_timestamp=True)
            stamp = time.time()
            if  abs(stamp - timestamp) < poseinfo_tolerence_period:
                now = rospy.get_rostime()
                msg = JointState()
                msg.header.stamp = now
                msg.header.frame_id = "From real-time state data"
                msg.name = joint_names
                msg.position = [0.0]*6
                for i, q in enumerate(joint[:6]):
                    msg.position[i] = q + joint_offsets.get(joint_names[i], 0.0)
                msg.velocity = [0.0]*6
                msg.effort = [0.0]*6
                pub_joint_states.publish(msg)
                self.set_joint_states(msg)
                # print "last_joint_states", msg
                tcp_force = [0.0]*6
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = now
                wrench_msg.wrench.force.x = tcp_force[0]
                wrench_msg.wrench.force.y = tcp_force[1]
                wrench_msg.wrench.force.z = tcp_force[2]
                wrench_msg.wrench.torque.x = tcp_force[3]
                wrench_msg.wrench.torque.y = tcp_force[4]
                wrench_msg.wrench.torque.z = tcp_force[5]
                pub_wrench.publish(wrench_msg)
            else:
                if not self.state_deadline_1:
                    self.state_deadline_1 = time.time()
                    if not self.state_deadline_2:
                        self.state_deadline_2 = time.time()
                else:
                    self.state_deadline_1 = time.time()
                if self.state_deadline_1 - self.state_deadline_2 > 2:
                    msg = JointState()
                    msg.header.stamp = rospy.get_rostime()
                    msg.header.frame_id = "From real-time state data"
                    msg.name = joint_names
                    msg.position = [0.0]*7
                    msg.velocity = [0.0]*6
                    msg.effort = [0.0]*6
                    pub_joint_states.publish(msg)
                    tcp_force = [0.0]*6
                    wrench_msg = WrenchStamped()
                    wrench_msg.header.stamp = rospy.get_rostime()
                    wrench_msg.wrench.force.x = self.tcp_force[0]
                    wrench_msg.wrench.force.y = self.tcp_force[1]
                    wrench_msg.wrench.force.z = self.tcp_force[2]
                    wrench_msg.wrench.torque.x = self.tcp_force[3]
                    wrench_msg.wrench.torque.y = self.tcp_force[4]
                    wrench_msg.wrench.torque.z = self.tcp_force[5]
                    pub_wrench.publish(wrench_msg)
                    self.state_deadline_1 = time.time()
                    self.state_deadline_2 = time.time()
                    print ("2222222222")
                    # ret = self._ArmCtrl.start_topic()
                    # print "ret",ret
                       
    # Returns the last JointState message sent out
    def get_joint_states(self):
        with self.joint_lock:
            return self.last_joint_states

    def set_joint_states(self, states):
        with self.joint_lock:
            self.last_joint_states = states

    def deg2rad(self, deglist):
        joint = [0]*7
        i=0
        # if len(deglist) == 6:
        for j in deglist[:6]:
            joint[i] = j/180*math.pi
            i = i+1
        return joint

    def rad2deg(self, radlist):
        joint = [0]*7
        i=0
        # if len(radlist) == 6:
        for j in radlist[:6]:
            joint[i] = j/math.pi*180
            i = i+1
        return joint
        
    # Waits until all threads have completed. Allows KeyboardInterrupt to occur
    def joinAll(self, threads):
        while any(t.isAlive() for t in threads):
            for t in threads:
                t.join(0.2)

    # Returns the duration between moving from point (index-1) to point
    # index in the given JointTrajectory
    def get_segment_duration(self, traj, index):
        if index == 0:
            return traj.points[0].time_from_start.to_sec()
        return (traj.points[index].time_from_start - traj.points[index-1].time_from_start).to_sec()

    # Reorders the JointTrajectory traj according to the order in
    # joint_names.  Destructive.
    def reorder_traj_joints(self, traj, joint_names):
        order = [traj.joint_names.index(j) for j in joint_names]

        new_points = []
        for p in traj.points:
            new_points.append(JointTrajectoryPoint(
                positions = [p.positions[i] for i in order],
                velocities = [p.velocities[i] for i in order] if p.velocities else [],
                accelerations = [p.accelerations[i] for i in order] if p.accelerations else [],
                time_from_start = p.time_from_start))
        traj.joint_names = joint_names
        traj.points = new_points

    def interp_cubic(self, p0, p1, t_abs):
        T = (p1.time_from_start - p0.time_from_start).to_sec()
        t = t_abs - p0.time_from_start.to_sec()
        q = [0] * 6
        qdot = [0] * 6
        qddot = [0] * 6
        for i in range(len(p0.positions)):
            a = p0.positions[i]
            b = p0.velocities[i]
            c = (-3*p0.positions[i] + 3*p1.positions[i] - 2*T*p0.velocities[i] - T*p1.velocities[i]) / T**2
            d = (2*p0.positions[i] - 2*p1.positions[i] + T*p0.velocities[i] + T*p1.velocities[i]) / T**3

            q[i] = a + b*t + c*t**2 + d*t**3
            qdot[i] = b + 2*c*t + 3*d*t**2
            qddot[i] = 2*c + 6*d*t
        return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))

    # 添加robot偏移量
    # joint_names: list of joints
    # returns: { "joint_name" : joint_offset }
    # shoulder
    def load_joint_offsets(self, joint_names):
        from lxml import etree
        robot_description = rospy.get_param("robot_description")
        doc = etree.fromstring(robot_description)
        # select only 'calibration_offset' elements whose parent is a joint
        # element with a specific value for the name attribute
        expr = "/robot/joint[@name=$name]/calibration_offset"
        print ("expr",expr)
        result = {}
        for joint in joint_names:
            joint_elt = doc.xpath(expr, name=joint)
            if len(joint_elt) == 1:
                calibration_offset = float(joint_elt[0].get("value"))
                result[joint] = calibration_offset
                rospy.loginfo("Found calibration offset for joint \"%s\": %.4f" % (joint, calibration_offset))
            elif len(joint_elt) > 1:
                rospy.logerr("Too many joints matched on \"%s\". Please report to package maintainer(s)." % joint)
            else:
                rospy.logwarn("No calibration offset for joint \"%s\"" % joint)
        return result

    # Returns (q, qdot, qddot) for sampling the JointTrajectory at time t.
    # The time t is the time since the trajectory was started.
    def sample_traj(self, traj, t):
        # First point
        if t <= 0.0:
            return copy.deepcopy(traj.points[0])
        # Last point
        if t >= traj.points[-1].time_from_start.to_sec():
            return copy.deepcopy(traj.points[-1])

        # Finds the (middle) segment containing t
        i = 0
        while traj.points[i+1].time_from_start.to_sec() < t:
            i += 1
        return self.interp_cubic(traj.points[i], traj.points[i+1], t)

    def traj_is_finite(self, traj):
        for pt in traj.points:
            for p in pt.positions:
                if math.isinf(p) or math.isnan(p):
                    return False
            for v in pt.velocities:
                if math.isinf(v) or math.isnan(v):
                    return False
        return True

    def has_limited_velocities(self, traj):
        for p in traj.points:
            for v in p.velocities:
                print("++++++++++++++++ v:",v)
                if math.fabs(v) > max_velocity:
                    return False
        return True

    def has_velocities(self, traj):
        for p in traj.points:
            if len(p.velocities) != len(p.positions):
                return False
        return True

    def within_tolerance(self, a_vec, b_vec, tol_vec):
        for a, b, tol in zip(a_vec, b_vec, tol_vec):
            if abs(a - b) > tol:
                return False
        return True

# joint_names: list of joints
#
# returns: { "joint_name" : joint_offset }
def load_joint_offsets(joint_names):
    from lxml import etree
    robot_description = rospy.get_param("robot_description")
    doc = etree.fromstring(robot_description)

    # select only 'calibration_offset' elements whose parent is a joint
    # element with a specific value for the name attribute
    expr = "/robot/joint[@name=$name]/calibration_offset"
    result = {}
    for joint in joint_names:
        joint_elt = doc.xpath(expr, name=joint)
        if len(joint_elt) == 1:
            calibration_offset = float(joint_elt[0].get("value"))
            result[joint] = calibration_offset
            rospy.loginfo("Found calibration offset for joint \"%s\": %.4f" % (joint, calibration_offset))
        elif len(joint_elt) > 1:
            rospy.logerr("Too many joints matched on \"%s\". Please report to package maintainer(s)." % joint)
        else:
            rospy.logwarn("No calibration offset for joint \"%s\"" % joint)
    return result

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node('ur_robot_driver', disable_signals=True)
    if rospy.get_param("use_sim_time", False):
        rospy.logwarn("use_sim_time is set!!!")
    proxy_ip =rospy.get_param("~proxy_ip", '169.254.53.102')
    adapter_name=rospy.get_param("~adapter_name",'ag_arm_1')

    # global prevent_programming
    # reconfigure_srv = Server(URDriverConfig, reconfigure_callback)
    print ("Current state:")
    # plan to a random location
    prefix = rospy.get_param("~prefix", "")
    print ("Setting prefix to %s" % prefix)
    global joint_names
    joint_names = [prefix + name for name in JOINT_NAMES]
    print ("joint_names",joint_names)
    # Reads the calibrated joint offsets from the URDF
    global joint_offsets
    joint_offsets = {"shoulder_pan_joint":0.0,"shoulder_lift_joint": 0.0,"elbow_joint":0.0,
    "wrist_1_joint":0.0,"wrist_2_joint":0.0,"wrist_3_joint":0.0 } #load_joint_offsets(joint_names)
    print ("joint_offsets", joint_offsets)

    if len(joint_offsets) > 0:
        rospy.loginfo("Loaded calibration offsets from urdf: %s" % joint_offsets)
    else:
        rospy.loginfo("No calibration offsets loaded from urdf")
    # Reads the maximum velocity
    #The max_velocity parameter is only used for debugging in the ur_driver. It's not related to actual velocity limits
    global max_velocity
    max_velocity = rospy.get_param("~max_velocity", MAX_VELOCITY) # [rad/s]
    rospy.loginfo("Max velocity accepted by ur_driver: %s [rad/s]" % max_velocity)
    # Reads the minimum payload
    global min_payload
    min_payload = rospy.get_param("~min_payload", MIN_PAYLOAD)
    # Reads the maximum payload
    global max_payload
    max_payload = rospy.get_param("~max_payload", MAX_PAYLOAD)
    rospy.loginfo("Bounds for Payload: [%s, %s]" % (min_payload, max_payload))
    robot = URTrajectoryFollower(proxy_ip=proxy_ip, adapter_name=adapter_name, goal_time_tolerance =4.0)

    robot.set_robot(robot=robot)
    robot.start()

    try:
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            # Checks for disconnect
    except KeyboardInterrupt:
        try:
            robot = URTrajectoryFollower(proxy_ip=proxy_ip, adapter_name=adapter_name,goal_time_tolerance =4.0)
            rospy.signal_shutdown("KeyboardInterrupt")
            if robot: robot.close()
        except:
            pass
        raise
    rospy.spin()
    roscpp_shutdown()



if __name__ == '__main__': main()

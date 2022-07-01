#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# from builtins import enumerate
import time
import threading
import os
import sys
import json
import Queue
import signal
import math,copy
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from moveit_commander import RobotCommander, MoveGroupCommander, MoveItCommanderException,roscpp_initialize, roscpp_shutdown
from moveit_ros_planning_interface import _moveit_robot_interface ,_moveit_move_group_interface
from moveit_msgs.msg import RobotState, RobotTrajectory, Constraints, MoveItErrorCodes
from visualization_msgs.msg import MarkerArray
import moveit_commander.conversions as conversions

import numpy as np
# from controller_msgs.msg import ControllerCmd,DataList
from geometry_msgs.msg import Pose, Quaternion, PoseArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix,concatenate_matrices,euler_from_matrix,rotation_matrix,identity_matrix
from copy import deepcopy
from math import pi
# from rlog import rlog
import subprocess
# log = rlog()

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../Universal_Robots_ROS_Driver/ur_robot_driver/scripts/lubancmd")

# from arm_interface import GripperInterface


class ControllerInterface(object):
    def __init__(self,proxy_ip, adapter_name='ag_arm_1', actuator_name='moveit',client_name='client_planner_1'):
        roscpp_initialize(sys.argv)
        rospy.init_node('controller_interface', anonymous = True, disable_signals = True)
        self.name_ = actuator_name
        self.adapter_name = adapter_name
        self.robot_handle = None
        self.arm_controller =None
        self.gripper_interface_1 = None
        self.tool_name = "tool0"
        time.sleep(5.0)
        self.__thread_cmd = threading.Thread(name = "roslaunch open", target = self._roslaunch_open)#, args = (self.arm_client,)
        self.__thread_cmd.daemon = True
        self.__thread_cmd.start()
        self.init_robot()
        # try:
        #     self.gripper_interface_1 = GripperInterface(proxy_ip= proxy_ip, adapter_name = adapter_name, actuator_name = "gripper",  client_name = "client_gripper_1")
        # except:
        #      print("{}: gripper_interface invalid".format(sys._getframe().f_code.co_name))
        # self.grip_pose_sub = rospy.Subscriber("/ag_moveit/controller_cmd", ControllerCmd, self.grip_pose_callback)

    def __del__(self):
        pass
        # launch_handle = subprocess.Popen(["kill -9 $(ps aux | grep ur3_bringup.launch | grep -v grep | awk '{print $2}')"],shell=True)
        # print("kill roslaunch ur_bringup ur3_bringup.launch:{}".format(launch_handle))
        # launch_handle = subprocess.Popen(["kill -9 $(ps aux | grep ur3_moveit_planning_execution.launch | grep -v grep | awk '{print $2}')"],shell=True)
        # print("kill roslaunch ur_bringup ur3_bringup.launch:{}".format(launch_handle))
        # launch_handle = subprocess.Popen(["kill -9 $(ps aux | grep rosmaster | grep -v grep | awk '{print $2}')"],shell=True)
    
    def _roslaunch_open(self):
        # launch_handle = subprocess.Popen(["roscore"],shell=True)
        pass
        # launch_handle = subprocess.Popen(["roslaunch ur_bringup ur3_bringup.launch"],shell=True)
        # print ("roslaunch ur_bringup ur3_bringup.launch:{}".format(launch_handle))

    def grip_pose_callback(self,data):
        if data.controller_cmd == "long_plan":
            print("exeutes long_plan")
            self.long_dist_plan(targetpos=data.targetpos,joints=None,speed=data.speed,acc=data.acc,eef_step=data.eef_step,end_effector_link=data.end_effector_link)
        elif data.controller_cmd == "mid_plan":
            print("exeutes mid_plan")
            self.mid_dist_plan(targetpos=data.targetpos,joints=None,speed=data.speed,acc=data.acc,eef_step=data.eef_step,end_effector_link=data.end_effector_link)
        elif data.controller_cmd == "short_plan":
            print("exeutes short_plan")
            self.short_dist_plan(targetpos=data.targetpos,targetpos_list=None,joints=None,speed=data.speed,acc=data.acc,eef_step=data.eef_step,end_effector_link=data.end_effector_link)
        elif data.controller_cmd == "turn_side":
            print("exeu release")
            self.fk_turn_plan(joints=data.joints,speed=data.speed,acc=data.acc)
        elif data.controller_cmd == "grip":
            print("exeu grip")
            print("data"),data 
            self.set_grip_beef(spose=data.spose,epose=data.epose,grippos=data.grippos,gripspeed=data.gripspeed,gripeffort=data.gripeffort,
                               speed=data.speed,acc=data.acc,eef_step=data.eef_step,end_effector_link=data.end_effector_link)
        elif data.controller_cmd == "move":
            print("exeu move")
            self.set_move_beef(spose=data.spose,epose=data.epose,
                               speed=data.speed,acc=data.acc,eef_step=data.eef_step,end_effector_link=data.end_effector_link)
        elif data.controller_cmd == "release":
            print("exeu release")
            self.set_release_beef(spose=data.spose,epose=data.epose,grippos=data.grippos,gripspeed=data.gripspeed,gripeffort=data.gripeffort,
                               speed=data.speed,acc=data.acc,eef_step=data.eef_step,end_effector_link=data.end_effector_link)
        elif data.controller_cmd == "gripper_control":
            print("exeu gripper_control")
            self.set_gripper_control(pos=data.grippos,speed=data.gripspeed,force = data.gripeffort)

    def init_robot(self):
        # self.adapter_name 
        arm_client = actionlib.SimpleActionClient(self.adapter_name + '/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for {}/scaled_pos_joint_traj_controller/follow_joint_trajectory server...".format(self.adapter_name))
        arm_client.wait_for_server()
        print("Connected to {}/scaled_pos_joint_traj_controller/follow_joint_trajectory server".format(self.adapter_name))
        # time.sleep(2)
        # launch_handle = subprocess.Popen(["roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch"],shell=True)
        # print("roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch:{}".format(launch_handle))
        # time.sleep(5)
        if self.robot_handle is None:
            try:
                self.robot_handle = RobotCommander("robot_description")
            except Exception, e:
                print("robot_handle is error:{}, {}. Can't start moveit controller...".format(e, self.robot_handle))
                print("Can't start moveit controller, exit...")
                sys.exit()
            group_name = self.robot_handle.get_group_names()
            print("group_name is:{}".format(group_name))

            # 初始化需要使用move group控制的机械臂中的arm group
            self.arm_controller = MoveGroupCommander("manipulator")

            # 设置目标位置所使用的参考坐标系
            base_frame = self.arm_controller.get_pose_reference_frame()
            print("current base_frame is:{}".format(base_frame))

            # 当运动规划失败后，允许重新规划
            self.arm_controller.allow_replanning(True)
            self.set_tolerance(0.001,0.0017)
            self.set_limited(1.0,1.0)
            self.set_effector_name('tool0')

            self.arm_controller.set_start_state( self.robot_handle.get_current_state())
            self.arm_controller.set_start_state_to_current_state()

            # if self.gripper_interface_1 is not None:
            #     self.gripper_interface_1.set_active()
        if self.robot_handle:
            return True
        else:
            return False

    def set_gripper_control(self, pos,speed=100,force=100):
        if self.gripper_interface_1 is not None:
            ret = self.gripper_interface_1.set_control(pos=int(pos), speed=int(speed), force=int(force))
            print("{} grippos:{} gripspeed:{} gripeffort:{} ret:{}".format(sys._getframe().f_code.co_name, pos, speed, force, ret))
        return True if ret == 0 else False

    # 设置误差范围,单位mm
    def set_tolerance(self,goal_position_tolerance=0.001,goal_orientation_tolerance=0.0017,goal_joint_tolerance= None):
        # 设置位置(单位：米)和姿态（单位：弧度)的允许误差
        self.arm_controller.set_goal_position_tolerance(goal_position_tolerance)
        self.arm_controller.set_goal_orientation_tolerance(goal_orientation_tolerance)
        if goal_joint_tolerance:
            self.arm_controller.set_goal_joint_tolerance(goal_joint_tolerance)

    # 设置的最大速度和加速度,单位mm
    def set_limited(self,speed=1.0,acc=0.5):
        self.arm_controller.set_max_velocity_scaling_factor(speed)
        self.arm_controller.set_max_acceleration_scaling_factor(acc)
        
    # 设置终端工具的名称 (note:move_group.set_end_effector_link() has no use?)
    def set_effector_name(self,end_effector_link='tool0'):
        self.arm_controller.set_end_effector_link(link_name = end_effector_link)

    # tanslate the list of tcp_euler to quatertion pose list
    def trans_eulerlist2quatelist(self,euler_list=None):
        quatlist = PoseArray()
        if not euler_list is None and len(euler_list)>0:
            euler_pose= [0]*6
            print("{}:dump euler_list->{}".format(sys._getframe().f_code.co_name, euler_list))
            for euler_pose in euler_list:
                print("euler_pose:{}".format(euler_pose))
                quatertion_pose = conversions.list_to_pose(euler_pose)
                quatlist.poses.append(quatertion_pose)
            return quatlist

    #用于长距离距离末端TCP反解路径规划
    def long_dist_plan(self,targetpos=None,joints=None,speed=1.0,acc=0.5,eef_step=0.001,end_effector_link="tool0"):
        # self.init_robot()
        self.arm_controller.set_start_state(self.robot_handle.get_current_state())
        self.arm_controller.set_start_state_to_current_state()
        print("{}:set_start_state_to_current_state befor plan:{}".format(sys._getframe().f_code.co_name, self.robot_handle.get_current_state()))
        self.set_limited(speed,acc)

        print("{}: targetpos:{} joints:{}".format(sys._getframe().f_code.co_name, targetpos, joints))
        print("{}: speed:{} acc:{} eef_step:{}".format(sys._getframe().f_code.co_name, speed, acc, eef_step))
        q_target_pose = None
        ret = MoveItErrorCodes.FAILURE

        if not joints is None:
            if type(joints) is list:
                q_target_pose = joints
        elif not targetpos is None:
            if type(targetpos) is list:
                q_target_pose= conversions.list_to_pose(targetpos)
                print("{}:q_target_pose after conversion:{}".format(sys._getframe().f_code.co_name, q_target_pose))
            elif type(targetpos) is Pose:
                q_target_pose = targetpos
            else:
                print("{}: targetpos is invalid".format(sys._getframe().f_code.co_name))
                return False
        else:
            print("{}: prameters invalid".format(sys._getframe().f_code.co_name))
            return False

        ret = self.arm_controller.go(q_target_pose)
        return True if ret == MoveItErrorCodes.SUCCESS else False

    #用于中等距离末端TCP反解路径规划
    def mid_dist_plan(self, targetpos = None, joints = None,speed = 1.0, acc=0.5, eef_step=0.001, end_effector_link="tool0"):
        # self.init_robot()
        self.arm_controller.set_start_state( self.robot_handle.get_current_state())
        self.arm_controller.set_start_state_to_current_state()
        self.set_limited(speed,acc)

        print("{}: targetpos:{} joints:{}".format(sys._getframe().f_code.co_name, targetpos, joints))
        print("{}: speed:{} acc:{} eef_step:{}".format(sys._getframe().f_code.co_name, speed, acc, eef_step))
        ret = MoveItErrorCodes.FAILURE
        q_target_pose = None

        if not joints is None:
            q_target_pose = joints
        elif not targetpos is None:
            if type(targetpos) is list:
                q_target_pose = conversions.list_to_pose(targetpos)
            elif type(targetpos) is Pose:
                q_target_pose = targetpos
            else:
                print("{}: targetpos invalid".format(sys._getframe().f_code.co_name))
                return False
        else:
            print("{}: prameters invalid".format(sys._getframe().f_code.co_name))
            return False

        attempts = 0
        max_attempts = 10
        while attempts < max_attempts:
            attempts += 1
            plan = self.arm_controller.plan(q_target_pose)
            print("targetpos attempts times:{}".format(attempts))
            if plan.joint_trajectory.points is not None and len(plan.joint_trajectory.points) > 0:
                retimed_plan = self.time_parameterization(plan,"time_optimal_trajectory_generation",speed,acc)
                if retimed_plan.joint_trajectory.points is not None and len(retimed_plan.joint_trajectory.points) > 0:
                    print("{}:Retimed plan is ok".format(sys._getframe().f_code.co_name))
                    ret = self.arm_controller.execute(retimed_plan)
                else:
                    ret = self.arm_controller.execute(plan)
                print("arm_controller.execute result:{}".format(ret))
                break
        if attempts >= max_attempts:
            print("plan failed with {} times retry".format(attempts))

        return True if ret == MoveItErrorCodes.SUCCESS else False

    def time_parameterization(self, plan, algorithm, speed=1.0, acc=0.5):
        ref_state = self.robot_handle.get_current_state()
        retimed_plan = self.arm_controller.retime_trajectory(
            ref_state, plan,
            velocity_scaling_factor = speed,
            acceleration_scaling_factor = acc,
            algorithm = algorithm)
        return retimed_plan

    #用于短距离末端TCP反解IK路径规划,euler_tcp_pose_list
    def short_dist_plan(self,targetpos=None,targetpos_list=None, joints=None, speed=1.0, acc=0.5,eef_step=0.001,end_effector_link="tool0"):
        # self.init_robot()
        self.arm_controller.set_start_state( self.robot_handle.get_current_state())
        self.arm_controller.set_start_state_to_current_state()
        self.set_limited(speed,acc)
        print("{}: targetpos:{} targetpos_list:{}".format(sys._getframe().f_code.co_name, targetpos, targetpos_list))
        print("{}: speed:{} acc:{} eef_step:{}".format(sys._getframe().f_code.co_name, speed, acc, eef_step))
        waypoints =[]
        # 设置机械臂轨迹约束值
        trajectory = RobotTrajectory()
        jump_threshold = 0
        eef_step = eef_step
        fraction = 0.0
        c = Constraints()
        ret = MoveItErrorCodes.FAILURE

        waypoints.append(deepcopy(self.arm_controller.get_current_pose(end_effector_link=end_effector_link).pose))
        if not targetpos is None:
            if type(targetpos) is list:
                pose = conversions.list_to_pose(targetpos)
                waypoints.append(pose)
        elif not targetpos_list is None:
            print("len of targetpos_list={}".format(len(targetpos_list)))
            pose_array = deepcopy(self.trans_eulerlist2quatelist(targetpos_list))
            print("pose_array={}".format(pose_array))
            pose = Pose()
            for pose in pose_array.poses:
                print("pose={}".format(pose))
                waypoints.append(pose)
        else:
            print("{}: prameters invalid".format(sys._getframe().f_code.co_name))
            return False

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        attempts = 0
        maxtries = 100
        print("dump waypoints:{}".format(waypoints))
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm_controller.compute_cartesian_path(waypoints, eef_step, fraction, True, path_constraints=c)
            attempts += 1
            if(attempts % 10 == 0):
                print("Still trying after {} attempts, fraction={}".format(attempts, fraction))

        if fraction == 1.0:
            print("Path computed successfully. Moving the arm.")
            retimed_plan = self.time_parameterization(plan,"time_optimal_trajectory_generation",speed,acc)
            if not retimed_plan.joint_trajectory.points is not None and len(retimed_plan.joint_trajectory.points) > 0:
                print("Retimed plan is ok")
                ret = self.arm_controller.execute(retimed_plan)
            else:
                ret = self.arm_controller.execute(plan)
            print("Path execution complete, result:{}".format(ret))
        else:
            print("Path planning failed with only {} success after {} attempts.".format(fraction, maxtries))

        return True if ret == MoveItErrorCodes.SUCCESS else False

    #FK规划,翻转
    def fk_turn_plan(self,joints=2*pi,speed=1.0, acc=0.5):
        # self.init_robot()
        self.arm_controller.set_start_state(self.robot_handle.get_current_state())
        self.arm_controller.set_start_state_to_current_state()
        self.set_limited(speed,acc)
        print("{}: joints:{}".format(sys._getframe().f_code.co_name, joints))
        print("{}: speed:{} acc:{}".format(sys._getframe().f_code.co_name, speed, acc))
        ret = MoveItErrorCodes.FAILURE

        # print("{}:speed:{} acc:{}".format(sys._getframe().f_code.co_name, speed, acc))
        # turn_joint= self.arm_controller.get_current_joint_values()
        # turn_joint[5] = turn_joint[5] - joint
        # if turn_joint[5] > 2*pi:
        #     turn_joint[5] = 2*pi - turn_joint[5]
        # elif turn_joint[5] < 2*pi:
        #     turn_joint[5] = -2*pi - turn_joint[5]

        ret = self.arm_controller.go(joints)
        return True if ret == MoveItErrorCodes.SUCCESS else False

    #夹取,末端tcp_pose_euler[x,y,z,rx,ry,rz],rx,ry,rz使用euler角, 1.spose代表准备夹取的位置 2.前移至牛排中心 3.抬起位置
    def set_grip_beef(self,spose=None,epose=None,grippos=0,gripspeed=100,gripeffort=100,joints=None,speed=1.0,acc=0.5,eef_step=0.001,end_effector_link="tool0",wait = True):
        """ Set the target of the group and then move the group to the specified target """
        _spose = None
        _epose = None
        ret = MoveItErrorCodes.FAILURE

        print("{}: speed:{} acc:{} eef_step:{}".format(sys._getframe().f_code.co_name, speed, acc, eef_step))
        if type(spose) is tuple or type(spose) is list:
            _spose = deepcopy(conversions.list_to_pose(spose))
        elif type(spose) is Pose:
            _spose = spose
        else:
            print("{}: spose:{} is invalid".format(sys._getframe().f_code.co_name, spose))

        if type(epose) is tuple or type(epose) is list:
            _epose = deepcopy(conversions.list_to_pose(epose))
        elif type(epose) is Pose:
            _epose= epose
        else:
            print("{}: epose:{} is invalid".format(sys._getframe().f_code.co_name, epose))

        #Move to spose
        if not _spose is None:
            ret = self.mid_dist_plan(targetpos=_spose,speed=speed,acc=acc,eef_step=eef_step,end_effector_link="tool0")
            if not ret:
                return False

        #Move to epose
        if not _epose is None:
            ret = self.mid_dist_plan(targetpos=_epose,speed=speed,acc=acc,eef_step=eef_step,end_effector_link="tool0")
            if not ret:
                return False

        #do grip
        ret = self.gripper_interface_1.set_control(pos=grippos, speed=gripspeed, force=gripeffort)
        print("{} grippos:{} gripspeed:{} gripeffort:{} ret:{}".format(sys._getframe().f_code.co_name, grippos, gripspeed, gripeffort, ret))
        return True if ret == 0 else False


    #移动,末端tcp_pose_euler[x,y,z,rx,ry,rz],rx,ry,rz使用euler角
    def set_move_beef(self,spose=None,epose=None,joints=None,speed=1.0,acc=0.5,eef_step=0.001,end_effector_link="tool0",wait = True):
        """ Set the target of the group and then move the group to the specified target """
        _spose = None
        _epose = None

        print("{}: speed:{} acc:{} eef_step:{}".format(sys._getframe().f_code.co_name, speed, acc, eef_step))
        if type(spose) is tuple or type(spose) is list:
            _spose = deepcopy(conversions.list_to_pose(spose))
        elif type(spose) is Pose:
            _spose = spose
        else:
            print("{}: spose:{} is invalid".format(sys._getframe().f_code.co_name, spose))

        if type(epose) is tuple or type(epose) is list:
            _epose = deepcopy(conversions.list_to_pose(epose))
        elif type(epose) is Pose:
            _epose= epose
        else:
            print("{}: epose:{} is invalid".format(sys._getframe().f_code.co_name, epose))

        #Move to spose
        if not _spose is None:
            ret = self.short_dist_plan(targetpos=_spose,joints=joints,speed=speed,acc=acc,eef_step=eef_step,end_effector_link="tool0")
            if not ret:
                return False

        #Move to epose
        if not _epose is None:
            ret = self.short_dist_plan(targetpos=_epose,joints=joints,speed=speed,acc=acc,eef_step=eef_step,end_effector_link="tool0")
            if not ret:
                return False

        return True

    #释放,末端tcp_pose_euler[x,y,z,rx,ry,rz],rx,ry,rz使用euler角, 1.spose代表准备释放的位置 2.释放3.mid后移至锅边 3.end抬起等待位置
    def set_release_beef(self,spose=None,epose=None,grippos=100,gripspeed=100, gripeffort=100, joints=None, speed=1.0, acc=0.5, eef_step=0.001, end_effector_link="tool0",wait = True):
        """ Set the target of the group and then move the group to the specified target """
        _spose = None
        _epose = None

        print("{}: speed:{} acc:{} eef_step:{}".format(sys._getframe().f_code.co_name, speed, acc, eef_step))
        if type(spose) is tuple or type(spose) is list:
            _spose = deepcopy(conversions.list_to_pose(spose))
        elif type(spose) is Pose:
            _spose = spose
        else:
            print("{}: spose:{} is invalid".format(sys._getframe().f_code.co_name, spose))

        if type(epose) is tuple or type(epose) is list:
            _epose = deepcopy(conversions.list_to_pose(epose))
        elif type(epose) is Pose:
            _epose= epose
        else:
            print("{}: spose:{} is invalid".format(sys._getframe().f_code.co_name, epose))

        #Move to spose
        if not _spose is None:
            ret = self.short_dist_plan(targetpos=_spose,joints=joints,speed=speed,acc=acc,eef_step=eef_step,end_effector_link="tool0")
            if not ret:
                return False

        #do grip
        ret = self.gripper_interface_1.set_control(pos=grippos, speed=gripspeed, force=gripeffort)
        print("{} grippos:{} gripspeed:{} gripeffort:{} ret:{}".format(sys._getframe().f_code.co_name, grippos, gripspeed, gripeffort, ret))
        if ret != 0:
            return False

        #Move to epose
        if not _epose is None:
            ret = self.mid_dist_plan(targetpos=_spose,joints=joints,speed=speed,acc=acc,eef_step=eef_step,end_effector_link="tool0")
            if not ret:
                return False
        return True

    #euler转旋转矢量
    def rodrigues2rotmat(self,vec):
        angle = np.sqrt(np.sum(vec**2))
        if angle == 0:
            return np.eye(3)
        axis = vec / angle
        mat = math.cos(angle) * np.eye(3) + (1 - math.cos(angle)) * axis.reshape(-1, 1) * axis + \
              math.sin(angle) * np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return mat

    def get_current_pose(self, end_effector_link="tool0"):
        return self.arm_controller.get_current_pose(end_effector_link=end_effector_link)
    
    # def list_to_joint(self, joint_list):
    #     joint_msg = JointState()
    #     if len(joint_list) == 6:
    #         joint_msg.position = copy(joint_list)
    #     else:
    #         raise MoveItCommanderException(
    #             "Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)"
    #         )
    #     return joint_msg

    def do_exeplan(self, targetpos=None, joint=None, speed=0.2, acc=0.1):
        ret = MoveItErrorCodes.FAILURE
        if not joint is None:
            if type(joint) is list:
                _epose = joint
                print("{}:joint to plan:{}".format(sys._getframe().f_code.co_name, joint))
                print("{}:joint_deg to plan:{}".format(sys._getframe().f_code.co_name, rad2deg(joint)))
        elif not targetpos is None:
            if type(targetpos) is list:
                _epose= conversions.list_to_pose(targetpos)
                print("{}:q_target_pose after conversion:{}".format(sys._getframe().f_code.co_name, targetpos))
            elif type(targetpos) is Pose:
                _epose = targetpos
                print("{}:q_target_pose to plan:{}".format(sys._getframe().f_code.co_name, targetpos))
            else:
                print("{}: targetpos is invalid".format(sys._getframe().f_code.co_name))
                return False
        else:
            print("{}: prameters invalid".format(sys._getframe().f_code.co_name))
            return False

        attempts = 0
        max_attempts = 10
        while attempts < max_attempts:
            attempts += 1
            print("++++++++++++++++++++ targetpos is:{}".format(_epose))
            plan = self.arm_controller.plan(_epose)
            print("targetpos attempts times:{}".format(attempts))
            if plan.joint_trajectory.points is not None and len(plan.joint_trajectory.points) > 0:
                retimed_plan = self.time_parameterization(plan,"time_optimal_trajectory_generation",speed,acc)
                if retimed_plan.joint_trajectory.points is not None and len(retimed_plan.joint_trajectory.points) > 0:
                    print("{}:Retimed plan is ok".format(sys._getframe().f_code.co_name))
                    ret = self.arm_controller.execute(retimed_plan)
                else:
                    ret = self.arm_controller.execute(plan)
                print("arm_controller.execute result:{}".format(ret))
                break
        if attempts >= max_attempts:
            print("plan failed with {} times retry".format(attempts))

        return True if ret == MoveItErrorCodes.SUCCESS else False

def deg2rad(deglist):
    joint = [0]*6
    i=0
    if len(deglist) == 6:
        for j in deglist:
            joint[i] = j/180*math.pi
            i = i+1
    return joint
def rad2deg(radlist):
    joint_deg = [0]*6
    i=0
    if len(radlist) == 6:
        for i, joint in enumerate(radlist):
            joint_deg[i] = joint/math.pi*180
    return joint_deg
def test1():
    # controller_interface = ControllerInterface(proxy_ip="169.254.53.102", adapter_name='ag_arm_1',actuator_name='moveit',client_name='client_moveit_1')
    # controller_interface.set_tolerance(0.001,0.0017)
    # tool_name = "tool0"
    # print("get_end_effector_link:{}".format(controller_interface.arm_controller.get_end_effector_link()))
    # cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link="tool0").pose
    # print("get_current_pose tool0:{}".format(cur_pose))
    # # cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link="tool0").pose
    # # print("get_current_pose_1 tool0:{}".format(cur_pose))
    # cur_rpy = controller_interface.arm_controller.get_current_rpy()
    # print("get_current_rpy:{}".format(cur_rpy))
    # cur_joint = controller_interface.arm_controller.get_current_joint_values()
    # print("get_current_joint_values:{}".format(cur_joint))
    # home  = [0, -1.57, 0, -1.57, 0, 0]
    # deg_home = [0,-90.0,0,-90.0,0,0]
    # start = [-0.018, 0.474, 0.095, 3.084, -0.639, 2.803]
    # start_joint_deg = [-90.2, -70.91, 53.09, -78.79, -87.50, 225.71]
    # start_joint = deg2rad(start_joint_deg)
    # print("start_joint:{}".format(start_joint))
    # ready_d1 = [-0.074, 0.500, -0.013, -3.071, -0.349, 2.609]
    # ready = [-0.124, 0.528, -0.013, -3.056, -0.891, 2.679]
    # grip_d1 = [-0.183, 0.566, -0.013, -2.911, -1.04, 2.499]
    # grip = [-0.231, 0.613, -0.010, -2.923, -1.101, 2.481]
    # end = [-0.246,0.598,0.201,1.327,3.797,0.473]
    # end_joint_deg = [-48.2, -70.91, 53.09, -78.79, -87.50, 125.71]
    # end_joint = deg2rad(end_joint_deg)
    # # controller_interface.set_limited(1.0,0.5)
    # controller_interface.arm_controller.set_start_state_to_current_state()
    # #euler_list = [[0] * 6 for i in range(3)]
    # euler_list = list()
    # # euler_list.append(start)
    # euler_list.append(ready_d1)
    # # euler_list.append(ready)
    # # euler_list.append(grip_d1)
    # euler_list.append(grip)
    # # euler_list.append(end)
    # print("euler_list:{}".format(euler_list))
    # # controller_interface.set_gripper_control(position=0)
    # controller_interface.set_limited(0.2,0.2)
    
    # is_plan_successed = controller_interface.mid_dist_plan(joints = start_joint, speed=0.2, acc=0.1)
    # print("+++++++++++ start_joint_deg:{}".format(start_joint_deg))
    # print("is_plan_successed start_joint:{} joints:{}".format(is_plan_successed,controller_interface.arm_controller.get_current_joint_values()))
    # print("22222222:{}".format(time.time()))
    # rospy.sleep(2)
    # is_plan_successed = controller_interface.mid_dist_plan(joints = end_joint, speed=0.2, acc=0.1)
    # print("+++++++++++ end_joint_deg:{}".format(end_joint_deg))
    # print("is_plan_successed end_joint:{} joints:{}".format(is_plan_successed,controller_interface.arm_controller.get_current_joint_values()))
    # print("111111111:{}".format(time.time()))


    controller_interface = ControllerInterface(proxy_ip="169.254.53.102", adapter_name='ag_arm_2',actuator_name='moveit',client_name='client_moveit_1')
    controller_interface.set_tolerance(0.001,0.0017)
    tool_name = "tool0"
    print("get_end_effector_link:{}".format(controller_interface.arm_controller.get_end_effector_link()))
    cur_pose = controller_interface.get_current_pose(end_effector_link=tool_name).pose
    print("get_current_pose tool0:{}".format(cur_pose))
    cur_rpy = controller_interface.arm_controller.get_current_rpy()
    print("get_current_rpy:{}".format(cur_rpy))
    cur_joint = controller_interface.arm_controller.get_current_joint_values()
    print("get_current_joint_values:{}".format(cur_joint))
    home  = [0, -1.57, 0, -1.57, 0, 0]
    deg_home = [0,-90.0,0,-90.0,0,0]
    start = [-0.018, 0.474, 0.095, 3.084, -0.639, 2.803]
    start_joint_deg = [-21.3, -87.18, -95.17, -185.08, -55.30, 336.01]
    start_joint = deg2rad(start_joint_deg)
    print("start_joint:{}".format(start_joint))
    ready_d1 = [-0.074, 0.500, -0.013, -3.071, -0.349, 2.609]
    ready = [-0.124, 0.528, -0.013, -3.056, -0.891, 2.679]
    grip_d1 = [-0.183, 0.566, -0.013, -2.911, -1.04, 2.499]
    grip = [-0.231, 0.613, -0.010, -2.923, -1.101, 2.481]
    end = [-0.246,0.598,0.201,1.327,3.797,0.473]
    end_joint_deg = [-15.27, -87.10, -61.48, -202.10, -83.63, 321.78]
    end_joint = deg2rad(end_joint_deg)
    # controller_interface.set_limited(1.0,0.5)
    controller_interface.arm_controller.set_start_state_to_current_state()
    #euler_list = [[0] * 6 for i in range(3)]
    euler_list = list()
    # euler_list.append(start)
    euler_list.append(ready_d1)
    # euler_list.append(ready)
    # euler_list.append(grip_d1)
    euler_list.append(grip)
    # euler_list.append(end)
    print("euler_list:{}".format(euler_list))
    # controller_interface.set_gripper_control(position=0)
    controller_interface.set_limited(0.2,0.2)
    
    # is_plan_successed = controller_interface.mid_dist_plan(joints = start_joint, speed=0.2, acc=0.1)
    # print("+++++++++++ start_joint_deg:{}".format(start_joint_deg))
    # print("is_plan_successed start_joint:{} joints:{}".format(is_plan_successed,controller_interface.arm_controller.get_current_joint_values()))
    # print("22222222:{}".format(time.time()))
    # rospy.sleep(1)
    # is_plan_successed = controller_interface.mid_dist_plan(joints = end_joint, speed=0.2, acc=0.1)
    # print("+++++++++++ end_joint_deg:{}".format(end_joint_deg))
    # print("is_plan_successed end_joint:{} joints:{}".format(is_plan_successed,controller_interface.arm_controller.get_current_joint_values()))
    # print("111111111:{}".format(time.time()))

    while not rospy.is_shutdown():
        rospy.sleep(1.0)
        # is_plan_successed = controller_interface.mid_dist_plan(joints = start_joint, speed=0.2, acc=0.1)
        # print("+++++++++++ start_joint_deg:{}".format(start_joint_deg))
        # print("is_plan_successed start_joint:{} joints:{}".format(is_plan_successed, controller_interface.arm_controller.get_current_joint_values()))
        # print("22222222:{}".format(time.time()))
        # rospy.sleep(1)
        # is_plan_successed = controller_interface.mid_dist_plan(joints = end_joint, speed=0.2, acc=0.1)
        # print("+++++++++++ end_joint_deg:{}".format(end_joint_deg))
        # print("is_plan_successed end_joint:{} joints:{}".format(is_plan_successed, controller_interface.arm_controller.get_current_joint_values()))
        # print("111111111:{}".format(time.time()))
    rospy.spin()
    roscpp_shutdown()



def trajectoryCallback(msg, controller_interface):
    # msg_local = deepcopy(msg)
    controller_interface.set_tolerance(0.001,0.0017)
    cur_joint = controller_interface.arm_controller.get_current_joint_values()
    print("get_current_joint_values:{}".format(cur_joint))
    controller_interface.arm_controller.set_start_state_to_current_state()
    controller_interface.set_limited(0.2,0.2)

    # for i, pose in enumerate(msg.poses):
    #     print("pose[{}]:{}".format(i,pose))
    #     print("x:{}, y:{}, z:{}".format(pose.position.x, pose.position.y, pose.position.z))
    #     print("x:{}, y:{}, z:{}, w:{}".format(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    #     if i == 0:
    # pose = list()

    toolname = controller_interface.arm_controller.get_end_effector_link()
    print("get_end_effector_link:{}".format(toolname))
    cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link=toolname).pose
    print("get_current_pose tool0:{}".format(cur_pose))
    cur_rpy = controller_interface.arm_controller.get_current_rpy()
    print("get_current_rpy:{}".format(cur_rpy))
    cur_joint = controller_interface.arm_controller.get_current_joint_values()
    print("get_current_joint_values:{}".format(cur_joint))

    pose1_tcp = [0.32955, -0.27566, 0.48207, 1.376, 1.163, 0.958]
    pose1_joint_deg = [-27.63, -119.67, -48.10, -20.14, 105.05, 174.94]
    pose1_joint = deg2rad(pose1_joint_deg)
    # controller_interface.do_exeplan(targetpos = pose1_tcp)
    # controller_interface.do_exeplan(joint = pose1_joint)
    while not rospy.is_shutdown():
        cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link=toolname).pose
        print("get_current_pose tool0:{}".format(cur_pose))
        cur_rpy = controller_interface.arm_controller.get_current_rpy()
        print("get_current_rpy:{}".format(cur_rpy))
        cur_joint = controller_interface.arm_controller.get_current_joint_values()
        print("get_current_joint_values:{}".format(cur_joint))
        print("get_current_joint_values:{}".format(rad2deg(cur_joint)))
        rospy.sleep(2)

    # pose2_tcp = [0.23770, -0.20873, 0.42463, 0.761, 1.310, 0.703]
    # pose2_joint_deg = [-25.99, -97.74, -98.96, 22.57, 110.71, 147.06]
    # pose2_joint = deg2rad(pose2_joint_deg)
    # # controller_interface.do_exeplan(targetpos = pose2_tcp)
    # controller_interface.do_exeplan(joint = pose2_joint)

    # msg_tmp = (PoseArray)(msg)
    # msg1 = PoseArray()
    # msg1 = Pose()
    # msg1.orientation

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node('controller_interface', anonymous = True, disable_signals = True)
    controller_interface = ControllerInterface(proxy_ip="169.254.53.102", adapter_name='ag_arm_1',actuator_name='moveit',client_name='client_moveit_1')
    msg_local = PoseArray()
    
    controller_interface.arm_controller.set_end_effector_link(link_name ="wrist_3_link")
    toolname = controller_interface.arm_controller.get_end_effector_link()
    print("get_end_effector_link:{}".format(toolname))
    cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link=toolname).pose
    print("get_current_pose {}:{}".format(toolname,cur_pose))
    cur_rpy = controller_interface.arm_controller.get_current_rpy(end_effector_link="wrist_3_link")
    print("get_current_rpy {}:{}".format("wrist_3_link",cur_rpy))
    cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link="flange").pose
    print("get_current_pose {}:{}".format("flange",cur_pose))
    cur_rpy = controller_interface.arm_controller.get_current_rpy(end_effector_link="flange")
    print("get_current_rpy {}:{}".format("flange",cur_rpy))
    cur_pose = controller_interface.arm_controller.get_current_pose(end_effector_link="tool0").pose
    print("get_current_pose {}:{}".format("tool0",cur_pose))
    cur_rpy = controller_interface.arm_controller.get_current_rpy(end_effector_link="tool0")
    print("get_current_rpy {}:{}".format("tool0",cur_rpy))
    cur_rpy = controller_interface.arm_controller.get_current_rpy(end_effector_link="base")
    print("get_current_rpy {}:{}".format("base",cur_rpy))
    cur_rpy = controller_interface.arm_controller.get_current_rpy(end_effector_link="base_link")
    print("get_current_rpy {}:{}".format("base_link",cur_rpy))
    cur_rpy = controller_interface.arm_controller.get_current_rpy(end_effector_link="base_link_inertia")
    print("get_current_rpy {}:{}".format("base_link_inertia",cur_rpy))
    cur_joint = controller_interface.arm_controller.get_current_joint_values()
    print("get_current_joint_values:{}".format(cur_joint))
    print("get_current_joint_values:{}".format(rad2deg(cur_joint)))
    rospy.sleep(2)

    while not rospy.is_shutdown():
        rospy.sleep(2)
    rospy.spin()
    roscpp_shutdown()

def test():
    r = UrxInterface(arm_ip = '192.168.2.101')
    # pose1=[-0.13231733694727157, -0.254666784404006, 0.4726276846593794, 1.667108334330631, -0.11176201274829108, -0.061063103131801764]
    # pose2=[-0.13237426157250276, -0.36524707026414893, 0.46949467850858884, 1.6177670092804897, -0.09470558616019928, -0.04579551271326503]
    # pose3=[0.08248608973388204, -0.38363091269370203, 0.45133984731957355, 1.6161561512449498, 0.3618938014566946, 0.37828534826934024]
    # pose4=[0.028864586146626087, -0.2561930562469201, 0.46028677413288926, 1.586865456678493, 0.41046443578189235, 0.4327255276604398]
    # pose5=[-0.09686867282776111, -0.2087294331910601, 0.4607760530441407, 1.5844723856664598, 0.015235698951981463, 0.0616993331587489]
    # pose6=[-0.15848692954119958, -0.30048160014951736, 0.4538643637071643, 1.748634864345248, -0.17906933648083978, -0.11561065811704914]
    pose1=[0.064, -0.302, 0.44, 0.99, 0.80, 0.13]
    pose2=[-0.050, -0.310, 0.421, 1.16, 0.48, -0.17]
    pose3=[-0.253, -0.147, 0.472, 0.322, -0.82, -1.37]
    pose4=[-0.191, -0.278, 0.414, 1.04, -0.386, -0.77]
    pose5=[-0.102, -0.308, 0.321, 1.47, 0.07, -0.08]
    pose6=[-0.131, -0.332, 0.238, 1.64, -0.09, 0.08]
    pose_list=[pose1,pose2,pose3,pose4,pose5]
    try:
        r.set_pose_curve(pose1,acc=0.4, speed=0.8)
    except r.RobotException as err:
        print("{}".format(err))
    rospy.sleep(1)
    r.set_pose_curve(pose2, acc=0.4, speed=0.8)
    rospy.sleep(1)
    # r.set_pose_curve(pose3, acc=0.4, speed=0.8)
    # rospy.sleep(1)
    # r.set_pose_curve(pose4, acc=0.4, speed=0.8)
    # rospy.sleep(1)
    # r.set_pose_curve(pose5, acc=0.4, speed=0.8)
    # rospy.sleep(1)
    # r.set_pose_curve(pose6, acc=0.4, speed=0.8)
    # rospy.sleep(1)
    r.robot.robotiq_activate(exe=True)
# r.movep_pose(pose = pose1,acc = 0.2, speed = 0.4, block = True)
# rospy.sleep(1)
# print("pose1:{}".format(r.get_pose()))
# r.movep_pose(pose = pose2, acc = 0.2, speed = 0.4, block = True)
# rospy.sleep(1)
# print("pose2:{}".format(r.get_pose()))
# r.movep_pose(pose = pose3,acc = 0.2, speed = 0.4, block = True)
# rospy.sleep(1)
# print("pose3:{}".format(r.get_pose()))

def disconnect(self):
    if self.__thread:
        self.__keep_running = False
        self.__thread.join()
        self.__thread = None
    if self.__sock:
        self.__sock.close()
        self.__sock = None
    self.last_state = None
    self.robot_state = self.DISCONNECTED

def exit_handle(signum, frame):
    print("You choose to stop me through ctrl+c.")
    sys.exit()

if __name__ == '__main__':
    # register exit function for exit by 'ctrl+c'
    signal.signal(signal.SIGINT, exit_handle)
    signal.signal(signal.SIGTERM, exit_handle)
    main()
    # test1()


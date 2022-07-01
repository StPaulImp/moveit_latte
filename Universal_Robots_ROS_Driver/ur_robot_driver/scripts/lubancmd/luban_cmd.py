#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import threading
import sys
import json
import signal
import paramiko

if sys.version_info[0] == 2:
    import Queue
elif sys.version_info[0] == 3:
    import queue
else:
    print("unknown python version")
    exit(-1)

import numpy as np
import rotation as Rotation

import autoconvarmplay

from arm_interface import SocketMsgInterface
from arm_interface import ArmInterface

HostIP = "169.254.53.102"


def start_luban_os(host_ip="192.168.1.102"):
    # 创建SSH对象
    ssh = paramiko.SSHClient()

    # 允许连接不在~/.ssh/known_hosts文件中的主机
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # 连接服务器
    ssh.connect(hostname=host_ip, port=22, username="root", password="root")

    # 执行命令，不要执行top之类的在不停的刷新的命令(可以执行多条命令，以分号来分隔多条命令)
    stdin, stdout, stderr = ssh.exec_command("sudo systemctl start luban")
    print("run sst command")


def stop_luban_os(host_ip="192.168.1.102"):
    # 创建SSH对象
    ssh = paramiko.SSHClient()

    # 允许连接不在~/.ssh/known_hosts文件中的主机
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # 连接服务器
    ssh.connect(hostname=host_ip, port=22, username="root", password="root")

    # 执行命令，不要执行top之类的在不停的刷新的命令(可以执行多条命令，以分号来分隔多条命令)
    stdin, stdout, stderr = ssh.exec_command("systemctl stop luban")
    print("run ssp command")


class LubanCmd:
    def __init__(self, host_ip="192.168.1.102"):
        self.socket_inf = SocketMsgInterface(socket_ip=host_ip)
        self.host_ip_ = host_ip

    def do_make(self, project_name):
        '''
        do_make
        do_clean
        '''
        ret = self.socket_inf.cmd_make(project_name)
        print("make end ret={}".format(ret))

    def reset(self):
        stop_luban_os(self.host_ip_)
        start_luban_os(self.host_ip_)
        time.sleep(5)

    # folder_path, the path of record data.
    def convert_to_armplay(self, folder_path):
        autoconvarmplay.auto_convert_to_armplay(folder_path)


def convert_to_armplay(folder_path, arm_type='ur', proxy_ip=None, with_graph=False):
    if 'ur' == arm_type:
        autoconvarmplay.auto_convert_to_armplay(folder_path, with_graph=with_graph)
    elif 'aubo' == arm_type:
        autoconvarmplay.auto_convert_to_armplay_aubo(folder_path, proxy_ip, with_graph=with_graph)
    elif 'jaka' == arm_type:
        autoconvarmplay.auto_convert_to_armplay_jaka(folder_path, proxy_ip, with_graph=with_graph)
    elif 'elite' == arm_type:
        autoconvarmplay.auto_convert_to_armplay_elite(folder_path, proxy_ip, with_graph=with_graph)


def luban_do_make():
    #start_luban_os()
    #time.sleep(5)
    #socket_inf = SocketMsgInterface(socket_ip=HostIP)
    #ret = socket_inf.cmd_make()
    luban_cmd = LubanCmd()
    ret = luban_cmd.do_make("do_clean")
    #stop_luban_os()
    print("make end ret={}".format(ret))


class ArmCtrl:
    def __init__(self, arm_name, host_ip=None):
        if host_ip == None:
            proxy_ip = HostIP
        else:
            proxy_ip = host_ip

        #stop_luban_os()
        #start_luban_os()
        #time.sleep(5)
        
        self.arm_interface_ = ArmInterface(proxy_ip, arm_name, 'arm', 'client' + arm_name)

        rtn, pose = self.arm_interface_.get_pos()

        if rtn == 0 and pose is not None:
           print("connect to Host success!")
        else:
           print("connect to Host fault!")
        self.arm_isMoving = False

    def get_pose(self, with_timestamp=False):
        if with_timestamp == True:
            ret, pose, timestamp = self.arm_interface_.get_cur_pose(with_timestamp)
            return ret, pose, timestamp
        else:
            ret, pose = self.arm_interface_.get_cur_pose(with_timestamp)
            return ret, pose

    def get_joints(self, with_timestamp=False):
        if with_timestamp == True:
            ret, joint, timestamp = self.arm_interface_.get_cur_joint(with_timestamp)
            return ret, joint, timestamp
        else:
            ret, joint = self.arm_interface_.get_cur_joint(with_timestamp)
            return ret, joint

    def movels(self, pose_list, a=0.01, v=0.01, radius=0.01, t=0):
        pose = [0, 0, 0, 0, 0, 0]
        pose_list_temp = [pose for i in range(len(pose_list))]
        for i in range(len(pose_list)):
            pose = pose_list[i].tolist()
            pose_list_temp[i] = pose[:]
        return self.arm_interface_.movels(pose_list_temp, a, v, t, radius, wait=False)

    def is_program_running(self):
        return self.arm_interface_.is_program_running()

    def movepose(self, tpose, acc=0.1, vel=0.05, time=0.0, wait=True,relative=False, threshold=None):
        if isinstance(tpose, list) is True:
            return self.arm_interface_.set_pose_curve(tpose, acc, vel, time, wait)
        else:
            return self.arm_interface_.set_pose_curve(tpose.tolist(), acc, vel, time, wait)

    def movejoint(self, tjoint, acc=0.1, vel=0.05, time=0.0, wait=True, relative=False, threshold=None):
        return self.arm_interface_.joint(tjoint, acc, vel, time, wait)

    def moveprocess(self, tpose, acc=0.1, vel=0.05, blend_radius=0, wait=True):
        if isinstance(tpose, list) is True:
            return self.arm_interface_.move_process(tpose, acc, vel, blend_radius, wait)
        else:
            return self.arm_interface_.move_process(tpose.tolist(), acc, vel, blend_radius, wait)

    def servojs(self, joint_list, time_list, lookaheadtime_list=None, gain_list=None, wait=False, relative=False, threshold=None):
        pose = [0, 0, 0, 0, 0, 0]
        pose_list_temp = [pose for i in range(len(joint_list))]
        for i in range(len(joint_list)):
            pose = joint_list[i].tolist()
            pose_list_temp[i] = pose[:]
        return self.arm_interface_.servojs(pose_list_temp, time_list[0], lookahead_time_list=lookaheadtime_list,
                                           gain_list=gain_list, wait=wait)

    def servops(self, pose_list, time_list, lookaheadtime_list=None, gain_list=None, wait=False, relative=False, threshold=None):
        pose = [0, 0, 0, 0, 0, 0]
        pose_list_temp = [pose for i in range(len(pose_list))]
        for i in range(len(pose_list)):
            pose = pose_list[i].tolist()
            pose_list_temp[i] = pose[:]

        # (self, pose_list, time_interval, lookahead_time_list, gain_list, isjoint=True, wait=True):

        return self.arm_interface_.servojs(pose_list_temp, time_list[0], lookaheadtime_list, gain_list,
                                           isjoint=False, wait=wait)

    def set_tool_offset(self, tool_offset):
        flange_tool = [0.0 for i in range(6)]
        # print("tool offset: {}".format(tool_offset))
        ret, org_pos = self.get_pose()
        # print("org pos:{}".format(org_pos))
        if 0 == ret:
            ret, cur_tcp_tool = self.get_tool_offset()
            if 0 == ret:
                if flange_tool != cur_tcp_tool:
                    # print("cur has been set tcp tool:{}".format(cur_tcp_tool))
                    org_conv_flange_tool_pos = self.tcp_trans_inv(org_pos, np.array(cur_tcp_tool))
                    org_conv_tool_pos = self.tcp_trans(org_conv_flange_tool_pos, tool_offset)
                else:
                    org_conv_tool_pos = self.tcp_trans(org_pos, tool_offset)
                org_conv_tool_pos = [round(i, 3) for i in org_conv_tool_pos]
                # print("org pos:{}, conv tool pos:{}".format(org_pos, org_conv_tool_pos))
            else:
                print("get cur tcp tool fault! ret={}".format(ret))
        else:
            print("get cur pos fault! ret={}".format(ret))

        if 0 == ret:
            ret = self.arm_interface_.set_tool_offset(tool_offset[0], tool_offset[1], tool_offset[2])

        if 0 == ret:
            break_flag = True
            before_time = time.clock()
            while break_flag:
                rtn, cur_pos = self.get_pose()
                cur_pos = [round(i, 3) for i in cur_pos]
                # print("cur pos:{}".format(cur_pos))
                if cur_pos == org_conv_tool_pos:
                    break_flag = False
                    # print("tcp tool set has been valid!")
                else:
                    cur_time = time.clock()
                    if before_time + 2 < cur_time:
                        print("tcp tool set has not valid, timeout!")
                        ret = -1
                        break_flag = False
                    else:
                        time.sleep(0.1)
        return ret

    def get_tool_offset(self):
        ret, tool_offset = self.arm_interface_.get_tool_offset()

        return ret, tool_offset

    def tcp_trans(self, flange_pose, tcp_vec):
        T_flange = np.zeros((4, 4))
        T_tcp = np.zeros((4, 4))
        tcp_pose=np.zeros(6)
        T_flange[0:3, 0:3] = Rotation.rodrigues2rotmat(flange_pose[3:])
        if isinstance(tcp_vec, list):
            T_tcp[0:3, 0:3] = Rotation.rodrigues2rotmat(np.array(tcp_vec[3:6]))
        else:
            T_tcp[0:3, 0:3] = Rotation.rodrigues2rotmat(tcp_vec[3:6])
        T_flange[0:3, 3] = flange_pose[0:3]
        T_tcp[0:3, 3] = tcp_vec[0:3]
        T_flange[3, 3] = 1
        T_tcp[3, 3] = 1
        T = np.dot(T_flange, T_tcp)
        tcp_pose[0:3] = T[0:3, 3]
        tcp_pose[3:] = Rotation.rotmat2rodrigues(T[0:3, 0:3])
        return tcp_pose

    def tcp_trans_inv(self, tool_pose, tcp_vec):
        T_tool = np.zeros((4, 4))
        T_tcp = np.zeros((4, 4))
        flange_pose=np.zeros(6)
        T_tool[0:3, 0:3] = Rotation.rodrigues2rotmat(tool_pose[3:6])
        T_tcp[0:3, 0:3] = Rotation.rodrigues2rotmat(tcp_vec[3:6])
        T_tool[0:3, 3] = tool_pose[0:3]
        T_tcp[0:3, 3] = tcp_vec[0:3]
        T_tool[3, 3] = 1
        T_tcp[3, 3] = 1
        T = np.dot(T_tool, np.linalg.inv(T_tcp))
        flange_pose[0:3] = T[0:3, 3]
        flange_pose[3:6] = Rotation.rotmat2rodrigues(T[0:3, 0:3])
        return flange_pose

    def start_teach_mode(self):
        ret = self.arm_interface_.start_teach_mode()
        return ret

    def stop_teach_mode(self):
        ret = self.arm_interface_.stop_teach_mode()

    def get_robot_serial_number(self):
        ret, serial_number = self.arm_interface_.get_robot_serial_number()
        return serial_number

    def rt_play_start(self, sample_time_ms):
        ret = 0
        if self.arm_isMoving == False:
            ret = self.arm_interface_.rt_play_start(sample_time_ms)
            self.arm_isMoving = True
        return ret

    def rt_play_end(self):
        ret = 0
        if self.arm_isMoving == True:
            ret = self.arm_interface_.rt_play_end()
            self.arm_isMoving = False
        return ret

    def rt_play_data(self, tjoint):
        ret = 0
        if self.arm_isMoving == True:
            if isinstance(tjoint, list) is True:
                ret = self.arm_interface_.rt_play_data(tjoint)
            else:
                ret = self.arm_interface_.rt_play_data(tjoint.tolist())
        return ret

def main():
    # luban_do_make()
    armctrl = ArmCtrl("ag_arm_1", host_ip='169.254.53.102')
    time.sleep(1)
    # tcp_tool = [0, -0.1425, 0.136, 0, 0, 0]
    tcp_tool = [0, 0, 0, 0, 0, 0]
    tcp_tool_np = np.array(tcp_tool)

    # armctrl.set_tool_offset(tcp_tool_np)

    tool_offset = armctrl.get_tool_offset()

    print(tool_offset)

    ret, cur_joint = armctrl.get_joints()

    print(cur_joint)

    armctrl.rt_play_start(10)

    time.sleep(0.5)

    for i in range(1000):
        move_joint = cur_joint
        move_joint[0] = cur_joint[0] + i*0.001
        armctrl.rt_play_data(move_joint)
        time.sleep(0.01)

    armctrl.rt_play_end()


    # armctrl.start_teach_mode()

    # luban_cmd = LubanCmd(HostIP)

    # luban_cmd.reset()

    while True:
        time.sleep(1)


def exit_handle(signum, frame):
    print("You choose to stop me through ctrl+c.")
    sys.exit()


if __name__ == "__main__":
    # register exit function for exit by 'ctrl+c'
    signal.signal(signal.SIGINT, exit_handle)
    signal.signal(signal.SIGTERM, exit_handle)

    main()

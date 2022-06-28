#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
this code is for convert pos or joint numpy array data to json string that
the luban can play. the numpy array data is after processing, transformed,
inversion, fine tune...
argv[1]: file path
argv[2]: type, 'joint' or 'pos'
"""

import sys
import os
import time
import json
import threading
import numpy as np
import collections
import matplotlib.pyplot as plt
import rotation as rotation
import math

from arm_interface import ArmInterface

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import Trajectory.mdl_basicfunctions as bs
# define play time interval, unit 's'
PLAY_INTERVAL = 0.01


def draw_wave(pose_list):

    x_list = []
    y_list = []
    z_list = []
    rx_list = []
    ry_list = []
    rz_list = []
    for pose in pose_list:
        x_list.append(pose[0])
        y_list.append(pose[1])
        z_list.append(pose[2])

        rx_list.append(pose[3])
        ry_list.append(pose[4])
        rz_list.append(pose[5])


    y_x = np.array(x_list)
    y_y = np.array(y_list)
    y_z = np.array(z_list)
    y_rx = np.array(rx_list)
    y_ry = np.array(ry_list)
    y_rz = np.array(rz_list)

    plt.figure(figsize=(16,8))
    plt.subplot(2,1,1)
    plt.plot(y_x, color='red', linewidth=1, linestyle='-', marker='.')
    plt.plot(y_y, color='blue', linewidth=1, linestyle='-', marker='.')
    plt.plot(y_z, color='black', linewidth=1, linestyle='-', marker='.')
    plt.subplot(2,1,2)
    plt.plot(y_rx, color='green', linewidth=1, linestyle='-', marker='.')
    plt.plot(y_ry, color='brown', linewidth=1, linestyle='-', marker='.')
    plt.plot(y_rz, color='gray', linewidth=1, linestyle='-', marker='.')
    plt.xlabel('time(ms)')
    plt.ylabel('m')
    #plt.ylim(-1.1,1.1)
   # plt.legend()
    plt.show()


def json_loads(json_str):
    # print "load json:",json_str
    try:
        strings = json.loads(json_str, encoding='utf-8')
    except:
        print("Parsing json string error! json_str:", json_str)
        return
    else:
        return strings


def json_dumps(dict_str):
    # print "dict:",dict_str
    try:
        # note [ensure_ascii=False] must be set, then utf-8 chinese code can be use
        json_str = json.dumps(dict_str, ensure_ascii=False)
    except:
        print("Dict to json error! dict:", dict_str)
        return
    else:
        return json_str


def tcp_trans(flange_pose, tcp_vec):
        T_flange = np.zeros((4, 4))
        T_tcp = np.zeros((4, 4))
        tcp_pose = np.zeros(6)
        T_flange[0:3, 0:3] = rotation.rodrigues2rotmat(flange_pose[3:6])
        T_tcp[0:3, 0:3] = rotation.rodrigues2rotmat(tcp_vec[3:6])
        T_flange[0:3, 3] = flange_pose[0:3]
        T_tcp[0:3, 3] = tcp_vec[0:3]
        T_flange[3, 3] = 1
        T_tcp[3, 3] = 1
        T = np.dot(T_flange, T_tcp)
        tcp_pose[0:3] = T[0:3, 3]
        tcp_pose[3:6] = rotation.rotmat2rodrigues(T[0:3, 0:3])
        return tcp_pose


def tcp_trans_inv(tool_pose, tcp_vec):
    T_tool = np.zeros((4, 4))
    T_tcp = np.zeros((4, 4))
    flange_pose = np.zeros(6)
    T_tool[0:3, 0:3] = rotation.rodrigues2rotmat(tool_pose[3:6])
    T_tcp[0:3, 0:3] = rotation.rodrigues2rotmat(tcp_vec[3:6])
    T_tool[0:3, 3] = tool_pose[0:3]
    T_tcp[0:3, 3] = tcp_vec[0:3]
    T_tool[3, 3] = 1
    T_tcp[3, 3] = 1
    T = np.dot(T_tool, np.linalg.inv(T_tcp))
    flange_pose[0:3] = T[0:3, 3]
    flange_pose[3:6] = rotation.rotmat2rodrigues(T[0:3, 0:3])
    return flange_pose


def file_conv(file_path, file_data_type, play_interval=0.01, lookahead_time=0.1, start_joint=None, start_tcp_tool=None, out_name=None, with_graph=True):
    np.set_printoptions(suppress=True)
    pose_list_np = np.loadtxt(file_path, delimiter=",")

    pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    (file, ext) = os.path.splitext(file_path)
    (path, filename) = os.path.split(file)
    record_name = filename

    if pose_list_np is not None:
        #print(pose_list_np)
        total_num = len(pose_list_np)
        pose_list = [pose for i in range(total_num)]
        # print pose_list
        for i in range(total_num):
            pose = [round(j, 5) for j in pose_list_np[i]]
            # print pose
            pose_list[i] = pose[:]
        # print pose_list

        # use collections for ensure the sort of dict
        record_dic = collections.OrderedDict()

        record_dic['name'] = file_path
        record_dic['datatime'] = time.strftime("%Y.%m.%d-%H:%M:%S", time.localtime())
        record_dic['interval'] = play_interval
        record_dic['lookahead'] = lookahead_time
        record_dic['count'] = total_num
        record_dic['startjoint'] = start_joint
        record_dic['starttcptool'] = start_tcp_tool
        if 'pos' == file_data_type:
            record_dic['tcplist'] = pose_list
        elif 'joint' == file_data_type:
            record_dic['jointlist'] = pose_list

        # print record_dic

        record_json = json_dumps(record_dic)

        # print record_json
        if out_name is not None:
            record_name = out_name

        record_json_file = open(path + '/' + record_name + '.armplay', 'w')
        record_json_file.write(record_json)
        record_json_file.close()

        if with_graph:
            draw_wave(pose_list)
        print("convert success, please check the file {0}.armplay".format(record_name))
    else:
        print("load file error!")


# read position offset and convert armplay flange center
def convc_armplay_flange(file_path, proxy_ip=None, arm_num=0, offset_x=0, offset_y=0, offset_z=0,
                                offset_eula_x=0, offset_eula_y=0, offset_eula_z=0, start_pos=None,
                                tcp_tool_offset=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                flange_center_rotate=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                play_interval=0.01,
                                output_path=None,
                                with_graph=True):

    if os.path.exists(file_path) is False:
        print("not find path!")
        exit(-1)

    if proxy_ip is None:
        proxy_ip = "127.0.0.1"

    if 0 == arm_num:
        arm_adapter = 'ag_arm_1'
    else:
        arm_adapter = 'ag_arm_2'

    print("connect to {}, proxy_ip={}, offset_x={}, offset_y={}, offset_z={}".format(arm_adapter, proxy_ip, offset_x, offset_y, offset_z))

    arm_interface_1 = ArmInterface(proxy_ip, arm_adapter, 'arm', 'client1' + arm_adapter)

    print("set tool offset, ".format(tcp_tool_offset))
    tcp_tool = tcp_tool_offset
    tcp_tool_np = np.array(tcp_tool)
    time.sleep(1)

    if start_pos is not None:
        pose_tcp_tool_inv_np = tcp_trans_inv(np.array(start_pos), tcp_tool_np)
        start_pos_flange_center = pose_tcp_tool_inv_np.tolist()
        # arm_interface_1.set_pose_curve(start_pos_inv)
        time.sleep(1)
        cur_pose = start_pos_flange_center[:]
    else:
        rtn, cur_pose = arm_interface_1.get_pos()

    print("cur pose={}".format(cur_pose))

    # get current joint and inverse joint with current pos
    ret, cur_joint = arm_interface_1.get_joint()

    print("cur joint={}".format(cur_joint))

    ret, joint = arm_interface_1.inverse_kinematic(cur_pose, cur_joint)

    print("inv joint={}".format(joint))

    cur_pose_tcp_tool_np = tcp_trans(np.array(cur_pose), tcp_tool_np)
    cur_pose_tcp_tool = cur_pose_tcp_tool_np.tolist()
    print("current pos base tcp tool:{}".format(cur_pose_tcp_tool))
    time.sleep(3)

    np.set_printoptions(suppress=True)
    pose_list_np = np.loadtxt(file_path, delimiter=",")

    pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    (file, ext) = os.path.splitext(file_path)
    (path, filename) = os.path.split(file)
    record_name = filename

    if pose_list_np is not None:
        # print(pose_list_np)
        total_num = len(pose_list_np)
        pose_list = [pose for i in range(total_num)]
        joint_list = [joint for i in range(total_num)]

        joint_ref = cur_joint[:]
        # print pose_list
        for i in range(total_num):
            # read offset pos with tcp tool
            pose = [round(j, 5) for j in pose_list_np[i]]

            # print pose
            # add start pos
            # pose[0] = pose[0] + start_pos[0]
            # pose[1] = pose[1] + start_pos[1]
            # pose[2] = pose[2] + start_pos[2]

            # add offset
            pose[0] = pose[0] + offset_x
            pose[1] = pose[1] + offset_y
            pose[2] = pose[2] + offset_z

            # convert pos with tcp tool to pos with flange center
            # print("pose with tcp tool:{}".format(pose))
            pose_tcp_tool_inv_np = tcp_trans_inv(np.array(pose), tcp_tool_np)
            pose_tcp_tool_inv = pose_tcp_tool_inv_np.tolist()
            pose = pose_tcp_tool_inv[:]

            # modified pos vector to current arm
            pose_vec = pose[:]
            pose_np = tcp_trans(np.array(pose_vec), np.array(flange_center_rotate))
            vec_inv = [pose_np[3], pose_np[4], pose_np[5]]

            pose[3] = vec_inv[0]
            pose[4] = vec_inv[1]
            pose[5] = vec_inv[2]

            pose = [round(j, 5) for j in pose]

            pose_list[i] = pose[:]

            ret, joint = arm_interface_1.inverse_kinematic(pose, joint_ref)

            joint_ref = joint[:]

            joint = [round(j, 5) for j in joint]

            joint_list[i] = joint[:]

            # print("pose:{}".format(pose))
            # print("joint:{}".format(joint))

        # print pose_list
        # print record_json

        # use collections for ensure the sort of dict
        record_dic = collections.OrderedDict()

        record_dic['name'] = file_path
        record_dic['datatime'] = time.strftime("%Y.%m.%d-%H:%M:%S", time.localtime())
        record_dic['interval'] = play_interval
        record_dic['count'] = total_num
        record_dic['tcplist'] = pose_list
        record_dic['jointlist'] = joint_list

        record_json = json_dumps(record_dic)

        if with_graph:
            # draw_wave(pose_list)
            draw_wave(joint_list)

        return record_json
    else:
        print("load file error!")


# read position offset and convc armplay flange
def convc_armplay_flange_jaka(file_path, proxy_ip=None, arm_num=0, offset_x=0, offset_y=0, offset_z=0,
                              offset_eula_x=0, offset_eula_y=0, offset_eula_z=0, start_pos=None,
                              tcp_tool_offset=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              flange_center_rotate=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              play_interval=0.01,
                              output_path=None,
                              with_graph=True):
    if os.path.exists(file_path) is False:
        print("not find path!")
        exit(-1)

    if proxy_ip is None:
        proxy_ip = "127.0.0.1"

    if 0 == arm_num:
        arm_adapter = 'ag_arm_1'
    else:
        arm_adapter = 'ag_arm_2'

    print("connect to {}, proxy_ip={}, offset_x={}, offset_y={}, offset_z={}".format(arm_adapter, proxy_ip, offset_x,
                                                                                     offset_y, offset_z))

    arm_interface_1 = ArmInterface(proxy_ip, arm_adapter, 'arm', 'client1' + arm_adapter)

    print("set tool offset, {}".format(tcp_tool_offset))
    tcp_tool = tcp_tool_offset
    tcp_tool_np = np.array(tcp_tool)
    time.sleep(1)

    if start_pos is not None:
        pose_tcp_tool_inv_np = tcp_trans_inv(np.array(start_pos), tcp_tool_np)
        start_pos_inv = pose_tcp_tool_inv_np.tolist()
        # arm_interface_1.set_pose_curve(start_pos_inv)
        time.sleep(1)
        cur_pose = start_pos_inv[:]
    else:
        rtn, cur_pose = arm_interface_1.get_pos()

    print("cur pose={}".format(cur_pose))

    ret, cur_joint = arm_interface_1.get_joint()

    print("cur joint={}".format(cur_joint))

    ret, joint = arm_interface_1.inverse_kinematic(cur_pose, cur_joint)

    print("inv joint={}".format(joint))

    cur_pose_tcp_tool_np = tcp_trans(np.array(cur_pose), tcp_tool_np)
    cur_pose_tcp_tool = cur_pose_tcp_tool_np.tolist()
    print("current pos base tcp tool:{}".format(cur_pose_tcp_tool))
    time.sleep(1)

    np.set_printoptions(suppress=True)
    pose_list_np = np.loadtxt(file_path, delimiter=",")

    pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    (file, ext) = os.path.splitext(file_path)
    (path, filename) = os.path.split(file)
    record_name = filename

    if pose_list_np is not None:
        print(pose_list_np)
        total_num = len(pose_list_np)
        pose_list = [pose for i in range(total_num)]
        joint_list = [joint for i in range(total_num)]

        joint_ref = cur_joint[:]
        # joint_ref = [0,1.57,-1.57,1.57,1.57,0]
        # print pose_list
        print("start convert ...")
        for i in range(total_num):
            pose = [round(j, 5) for j in pose_list_np[i]]
            # print pose
            pose[0] = pose[0] + offset_x
            pose[1] = pose[1] + offset_y
            pose[2] = pose[2] + offset_z

            # pose[0] = pose[0] + cur_pose_tcp_tool[0]
            # pose[1] = pose[1] + cur_pose_tcp_tool[1]
            # pose[2] = pose[2] + cur_pose_tcp_tool[2]

            pose_vec = pose[:]
            pose_np = tcp_trans(np.array(pose_vec), np.array(flange_center_rotate))
            vec_inv = [pose_np[3], pose_np[4], pose_np[5]]
            # print("vec_inv:{}".format(vec_inv))

            # pose[3] = vec_inv[0]
            # pose[4] = vec_inv[1]
            # pose[5] = vec_inv[2]

            # convert pos with tcp tool to pos with flange center
            # print("pose with tcp tool:{}".format(pose))
            pose_tcp_tool_inv_np = tcp_trans_inv(np.array(pose), tcp_tool_np)
            pose_tcp_tool_inv = pose_tcp_tool_inv_np.tolist()
            pose = pose_tcp_tool_inv[:]

            pose[3] = vec_inv[0]
            pose[4] = vec_inv[1]
            pose[5] = vec_inv[2]

            # pose = [round(j, 5) for j in pose]

            pose_list[i] = pose[:]

            ret, joint = arm_interface_1.inverse_kinematic(pose, joint_ref)
            joint_ref = joint[:]

            if arm_adapter == 'ag_arm_2':
                if joint[0] < 0.0:
                    joint[0] = math.pi + math.pi + joint[0]
                    pass

            if arm_adapter == 'ag_arm_1':
                if joint[0] < 0.0:
                    joint[0] = math.pi + math.pi + joint[0]
                    pass
            # joint = [round(j, 5) for j in joint]
            # joint[5] = joint[5] - math.pi/4 + math.pi

            joint_list[i] = joint[:]

            # print("pose:{}".format(pose))
            # print("joint:{}".format(joint))
            # print("count:{}".format(i))

        # print pose_list
        # print record_json

        # joint list resample
        joint_list_resample = joint_resample(joint_list, play_interval, 0.008, 2)
        total_num = len(joint_list_resample)
        for i in range(total_num):
            joint_list_resample[i] = [round(j, 5) for j in joint_list_resample[i]]

        # use collections for ensure the sort of dict
        record_dic = collections.OrderedDict()

        record_dic['name'] = record_name
        record_dic['datatime'] = time.strftime("%Y.%m.%d-%H:%M:%S", time.localtime())
        record_dic['interval'] = 0.008
        record_dic['count'] = total_num
        record_dic['tcplist'] = pose_list
        record_dic['jointlist'] = joint_list_resample

        # print record_dic

        record_json = json_dumps(record_dic)

        if with_graph:
            draw_wave(joint_list_resample)

        return record_json
    else:
        print("load file error!")


def joint_resample(joint_list, org_interval, new_interval, filter_level):
    print('resample joint list, org interval={}, new interval={}, filter level={}'.format(
          org_interval, new_interval, filter_level))

    total_num = len(joint_list)

    print('read joint list len={}'.format(total_num))

    #filter_level = 2 # 0 -- no filter at all ; 1 -- low level ; 2 -- default level ; 3 -- high filter level
    joints_resampled = bs.JointReSample(joint_list, org_interval, new_interval, filter_level)

    total_num = len(joints_resampled)

    print('resample joint list len={}'.format(total_num))

    return joints_resampled




if __name__ == "__main__":
    print("play file convert after processing")
    print("first argv: file path, second argv: data type, 'joint' or 'pos'")

    file_path = 'record1.txt'
    file_data_type = 'pos'
    if len(sys.argv) > 1:
        file_path = sys.argv[1]

    if len(sys.argv) > 2:
        file_data_type = sys.argv[2]

    if len(sys.argv) > 3:
        play_interval = sys.argv[3]
    else:
        play_interval = 0.01

    if len(sys.argv) > 4:
        lookahead_time = sys.argv[4]
    else:
        lookahead_time = 0.1

    if os.path.exists(file_path) is False:
        print("not find path!")
        exit(-1)
    file_conv(file_path, file_data_type, play_interval, lookahead_time)

#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
this code is for auto convert track data in assign folder to '.armplay' data, to use playfileconv,

the code will scan assign folder, and conv 'cup_pose.txt' and 'milk_pose.txt' file to .armplay, depend on 'config.txt' file

folder name format {time stamp}_{name}, example 20200820_heart
"""

import sys
import os
import time
import json
import threading
import numpy as np
import collections
import shutil
import math

import playfileconv


def parse_config_file(config_path):
    f_in = open(config_path, "r")

    json_string = f_in.read()

    config_org_dic = json.loads(json_string)  # type: str

    # print(config_org_dic)

    cup_tcp_tool = config_org_dic['tcp_set']['cup']
    milk_tcp_tool = config_org_dic['tcp_set']['milk']

    cup_tcp_tool[3:] = [0.0, 0.0, 0.0]
    milk_tcp_tool[3:] = [0.0, 0.0, 0.0]

    config_dic = {'milk_start_joint': config_org_dic['milk_arm_start_joints'],
                  'cup_start_joint': config_org_dic['cup_arm_start_joints'],
                  'cup_tcp_tool': cup_tcp_tool,
                  'milk_tcp_tool': milk_tcp_tool,
                  'milk_lookahead_time': config_org_dic['lookahead_time'],
                  'cup_lookahead_time': 0.1,
                  'interval': 0.01}

    print(config_dic)

    return config_dic


def auto_convert_to_armplay(file_path, with_graph=True):
    print("=={}==".format(file_path))
    if file_path is None:
        print('please input file path')
        return

    if '/' == file_path[-1]:
        file_path = file_path[:-1]
        print(file_path)

    file_paths = os.listdir(file_path)
    #print(file_paths)

    config_file_path = file_path + '/config.txt'
    cup_pose_file_path = file_path + '/cup-pose.txt'
    milk_pose_file_path = file_path + '/milk-pose.txt'

    if os.path.exists(file_path) is False:
        print("not find path! {}".format(file_path))
        return

    file_path_split = file_path.split("-", 1)

    file_data_time = file_path_split[0]
    file_name = file_path_split[1]

    # print(file_path_split)

    config_dic = parse_config_file(config_file_path)

    # milk
    play_interval = config_dic['interval']
    lookahead_time = config_dic['milk_lookahead_time']
    tcp_tool_offset = config_dic['milk_tcp_tool']
    start_joint = config_dic['milk_start_joint']
    playfileconv.file_conv(milk_pose_file_path, 'pos', play_interval=play_interval, lookahead_time=lookahead_time,
                           start_joint=start_joint, start_tcp_tool=tcp_tool_offset, out_name='milk_' + file_name,
                           with_graph=with_graph)

    play_interval = config_dic['interval']
    lookahead_time = config_dic['cup_lookahead_time']
    tcp_tool_offset = config_dic['cup_tcp_tool']
    start_joint = config_dic['cup_start_joint']
    playfileconv.file_conv(cup_pose_file_path, 'pos', play_interval=play_interval, lookahead_time=lookahead_time,
                           start_joint=start_joint, start_tcp_tool=tcp_tool_offset, out_name='cup_' + file_name,
                           with_graph=with_graph)


def auto_convert_to_armplay_aubo(file_path, proxy_ip, with_graph=True):
    print("=={}==".format(file_path))
    if file_path is None:
        print('please input file path')
        return

    if '/' == file_path[-1]:
        file_path = file_path[:-1]
        print(file_path)

    file_paths = os.listdir(file_path)
    #print(file_paths)

    flange_center_rotate_aubo = [0.0, 0.0, 0.0, 0.0, 0.0, math.pi]

    config_file_path = file_path + '/config.txt'
    cup_pose_file_path = file_path + '/cup-pose.txt'
    milk_pose_file_path = file_path + '/milk-pose.txt'

    if os.path.exists(file_path) is False:
        print("not find path! {}".format(file_path))
        return

    file_path_split = file_path.split("-", 1)

    file_data_time = file_path_split[0]
    file_name = file_path_split[1]

    # print(file_path_split)

    config_dic = parse_config_file(config_file_path)

    # milk
    arm_num = 1
    play_interval = config_dic['interval']
    lookahead_time = config_dic['milk_lookahead_time']
    tcp_tool_offset = config_dic['milk_tcp_tool']
    start_joint = config_dic['milk_start_joint']
    record_json = playfileconv.convc_armplay_flange(milk_pose_file_path, proxy_ip=proxy_ip, arm_num=arm_num,
                                                    offset_x=0.065,
                                                    tcp_tool_offset=tcp_tool_offset,
                                                    flange_center_rotate=flange_center_rotate_aubo,
                                                    with_graph=with_graph)

    output_path_name = file_path + '/' + 'milk_' + file_name + '.armplay'
    record_json_file = open(output_path_name, 'w')
    record_json_file.write(record_json)
    record_json_file.close()

    print("convert milk arm success")

    # cup
    arm_num = 0
    play_interval = config_dic['interval']
    lookahead_time = config_dic['cup_lookahead_time']
    tcp_tool_offset = config_dic['cup_tcp_tool']
    start_joint = config_dic['cup_start_joint']
    record_json = playfileconv.convc_armplay_flange(cup_pose_file_path, proxy_ip=proxy_ip, arm_num=arm_num,
                                                    offset_x=-0.065,
                                                    tcp_tool_offset=tcp_tool_offset,
                                                    flange_center_rotate=flange_center_rotate_aubo,
                                                    with_graph=with_graph)

    output_path_name = file_path + '/' + 'cup_' + file_name + '.armplay'
    record_json_file = open(output_path_name, 'w')
    record_json_file.write(record_json)
    record_json_file.close()

    print("convert cup arm success")


def auto_convert_to_armplay_elite(file_path, proxy_ip, with_graph=True):
    print("=={}==".format(file_path))
    if file_path is None:
        print('please input file path')
        return

    if '/' == file_path[-1]:
        file_path = file_path[:-1]
        print(file_path)

    file_paths = os.listdir(file_path)
    #print(file_paths)

    flange_center_rotate_elite = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    config_file_path = file_path + '/config.txt'
    cup_pose_file_path = file_path + '/cup-pose.txt'
    milk_pose_file_path = file_path + '/milk-pose.txt'

    if os.path.exists(file_path) is False:
        print("not find path! {}".format(file_path))
        return

    file_path_split = file_path.split("-", 1)

    file_data_time = file_path_split[0]
    file_name = file_path_split[1]

    # print(file_path_split)

    config_dic = parse_config_file(config_file_path)

    # milk
    arm_num = 1
    play_interval = config_dic['interval']
    lookahead_time = config_dic['milk_lookahead_time']
    tcp_tool_offset = config_dic['milk_tcp_tool']
    start_joint = config_dic['milk_start_joint']
    record_json = playfileconv.convc_armplay_flange(milk_pose_file_path, proxy_ip=proxy_ip, arm_num=arm_num,
                                                    offset_x=0.065,
                                                    tcp_tool_offset=tcp_tool_offset,
                                                    flange_center_rotate=flange_center_rotate_elite,
                                                    with_graph=with_graph)

    output_path_name = file_path + '/' + 'milk_' + file_name + '.armplay'
    record_json_file = open(output_path_name, 'w')
    record_json_file.write(record_json)
    record_json_file.close()

    print("convert milk arm success")

    # cup
    arm_num = 0
    play_interval = config_dic['interval']
    lookahead_time = config_dic['cup_lookahead_time']
    tcp_tool_offset = config_dic['cup_tcp_tool']
    start_joint = config_dic['cup_start_joint']
    record_json = playfileconv.convc_armplay_flange(cup_pose_file_path, proxy_ip=proxy_ip, arm_num=arm_num,
                                                    offset_x=-0.065,
                                                    tcp_tool_offset=tcp_tool_offset,
                                                    flange_center_rotate=flange_center_rotate_elite,
                                                    with_graph=with_graph)

    output_path_name = file_path + '/' + 'cup_' + file_name + '.armplay'
    record_json_file = open(output_path_name, 'w')
    record_json_file.write(record_json)
    record_json_file.close()

    print("convert cup arm success")


def auto_convert_to_armplay_jaka(file_path, proxy_ip, with_graph=True):
    print("=={}==".format(file_path))
    if file_path is None:
        print('please input file path')
        return

    if '/' == file_path[-1]:
        file_path = file_path[:-1]
        print(file_path)

    file_paths = os.listdir(file_path)
    print(file_paths)

    flange_center_rotate_jaka = [0.0, 0.0, 0.0, 0.0, 0.0, math.pi*3/4]

    config_file_path = file_path + '/config.txt'
    cup_pose_file_path = file_path + '/cup-pose.txt'
    milk_pose_file_path = file_path + '/milk-pose.txt'

    if os.path.exists(file_path) is False:
        print("not find path! {}".format(file_path))
        return

    file_path_split = file_path.split("-", 1)

    file_data_time = file_path_split[0]
    file_name = file_path_split[1]

    # print(file_path_split)

    config_dic = parse_config_file(config_file_path)

    # milk
    arm_num = 1
    play_interval = config_dic['interval']
    lookahead_time = config_dic['milk_lookahead_time']
    tcp_tool_offset = config_dic['milk_tcp_tool']
    start_joint = config_dic['milk_start_joint']
    record_json = playfileconv.convc_armplay_flange_jaka(milk_pose_file_path, proxy_ip=proxy_ip, arm_num=arm_num,
                                                    offset_x=0.065,
                                                    tcp_tool_offset=tcp_tool_offset,
                                                    flange_center_rotate=flange_center_rotate_jaka,
                                                    with_graph=with_graph)

    output_path_name = file_path + '/' + 'milk_' + file_name + '.armplay'
    record_json_file = open(output_path_name, 'w')
    record_json_file.write(record_json)
    record_json_file.close()

    print("convert milk arm success")

    # cup
    arm_num = 0
    play_interval = config_dic['interval']
    lookahead_time = config_dic['cup_lookahead_time']
    tcp_tool_offset = config_dic['cup_tcp_tool']
    start_joint = config_dic['cup_start_joint']
    record_json = playfileconv.convc_armplay_flange_jaka(cup_pose_file_path, proxy_ip=proxy_ip, arm_num=arm_num,
                                                    offset_x=-0.065,
                                                    tcp_tool_offset=tcp_tool_offset,
                                                    flange_center_rotate=flange_center_rotate_jaka,
                                                    with_graph=with_graph)

    output_path_name = file_path + '/' + 'cup_' + file_name + '.armplay'
    record_json_file = open(output_path_name, 'w')
    record_json_file.write(record_json)
    record_json_file.close()

    print("convert cup arm success")


if __name__ == "__main__":

    folder_path = None
    arm_type = None
    proxy_ip = '127.1.0.0'
    with_graph = True

    print('params [folder] [arm type] [ip] [with graph, True OR False]')

    if len(sys.argv) > 1:
        folder_path = sys.argv[1]

    if len(sys.argv) > 2:
        arm_type = sys.argv[2]

    if len(sys.argv) > 3:
        proxy_ip = sys.argv[3]

    if len(sys.argv) > 4:
        if sys.argv[4] == 'True':
            with_graph = True
        else:
            with_graph = False

    if folder_path is None:
        print('please input folder path')
        exit(-1)

    if arm_type is None:
        print('please input arm type [ur] [aubo] [jaka] [elite]')
        exit(-1)

    print("folder path:{} , arm type:{} , ip:{} , with graph:{}".format(folder_path, arm_type, proxy_ip, with_graph))

    if 'ur' == arm_type:
        auto_convert_to_armplay(folder_path, with_graph=with_graph)
    elif 'aubo' == arm_type:
        auto_convert_to_armplay_aubo(folder_path, proxy_ip, with_graph=with_graph)
    elif 'jaka' == arm_type:
        auto_convert_to_armplay_jaka(folder_path, proxy_ip, with_graph=with_graph)
    elif 'elite' == arm_type:
        auto_convert_to_armplay_elite(folder_path, proxy_ip, with_graph=with_graph)

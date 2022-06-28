#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import threading
import sys
import json
import signal

if sys.version_info[0] == 2:
    import Queue
elif sys.version_info[0] == 3:
    import queue
else:
    print("unknown python version")
    exit(-1)

import numpy as np

from proxy_client import ProxyClient, PS_Socket, Socket

ACTUATOR_NAME = "arm"

MSG_ACK_TIMEOUT_MS = 1000

MSG_WAIT_INFINITE = 5 #999999

if sys.version_info < (3, 0):
    STRING_TYPE = (str, unicode)
else:
    STRING_TYPE = str


class MsgCondition:
    def __init__(self, seq, adapter_name, cmd):
        self.seq_ = seq
        self.adapter_name_ = adapter_name
        self.cmd_ = cmd
        self.ack_event_ = threading.Event()
        self.res_event_ = threading.Event()
        self.status = "init"
        self.result_data = None
        self.is_has_handle = False
        self.code = 0
        self.codestring = ''


class ActuatorInterface:
    def __init__(self, client_name, proxy_ip=None):
        self.name_ = client_name
        self.actuator_name_ = None
        self.adapter_name_ = None
        self.core_name_ = None
        self.proxy_ip_ = proxy_ip
        self.condition_ = threading.Condition()
        self.msg_cond_list_lock_ = threading.Lock()
        self.msg_cond_list_ = []
        self.send_seq_ = 0
        self.send_seq_lock_ = threading.Lock()
        self.topic_list_ = []

        self.proxy_ = ProxyClient(client_name, self, self.proxy_callback, proxy_addr=proxy_ip)
        self.proxy_.connect()
        self.topic_socket_ = PS_Socket(PS_Socket.SUB, proxy_ip, self.subscribe_callback)

        # print("{}:connect to proxy [{}] success!".format(self.name_, self.proxy_ip_))

    def set_target(self, actuator_name, core_name=None):
        self.actuator_name_ = actuator_name
        self.core_name_ = core_name

    def set_target_direct(self, actuator_name, adapter_name):
        self.actuator_name_ = actuator_name
        self.adapter_name_ = adapter_name

    def proxy_callback(self, this_ins, msg):
        # print("{}:client recv: {}".format(self.name_, msg))
        adapter_name = self.get_dict_key_value(msg, 'cmdsrc', STRING_TYPE)

        if adapter_name is None:
            print("adapter name error!")
            return

        cmd = self.get_dict_key_value(msg, 'cmd', STRING_TYPE)
        if cmd is None:
            print("cmd error")
            return

        params = self.get_params(msg)

        seq = self.get_dict_key_value(params, 'seq', int)

        if seq is None:
            print("seq error")
            return

        type = self.get_dict_key_value(params, 'type', STRING_TYPE)

        if type is None:
            print("type error")
            return

        code = self.get_dict_key_value(params, 'code', int)
        if code is None:
            print("code error")
            return

        codestring = self.get_dict_key_value(params, 'codestring', STRING_TYPE)
        if codestring is None:
            codestring = ''

        data = self.get_dict_key_value(params, 'data')

        self.msg_cond_list_lock_.acquire()
        for msg_cond in self.msg_cond_list_:
            # print(msg_cond.seq_, msg_cond.is_has_handle, type, msg_cond.adapter_name_, msg_cond.cmd_)
            if msg_cond.seq_ == seq and type == 'ack' \
                    and msg_cond.adapter_name_ == adapter_name\
                    and msg_cond.cmd_ == cmd\
                    and msg_cond.is_has_handle is False:
                msg_cond.status = "ack"
                msg_cond.ack_event_.set()
            elif msg_cond.seq_ == seq and type == 'res' \
                    and msg_cond.adapter_name_ == adapter_name\
                    and msg_cond.cmd_ == cmd\
                    and msg_cond.is_has_handle is False:
                # print("get cond")
                msg_cond.result_data = data
                msg_cond.is_has_handle = True
                msg_cond.status = "res"
                msg_cond.code = code
                msg_cond.codestring = codestring
                msg_cond.res_event_.set()
            else:
                # print("unknow replay type!")
                pass
        self.msg_cond_list_lock_.release()

    def subscribe_callback(self, args, topic, content):
        # print("get topic:", content)
        for topic_tup in self.topic_list_:
            if topic == topic_tup[0]:
                topic_tup[1](content)

    # value_type: int, float, str, bool, list, dict
    def get_dict_key_value(self, dict_ins, key, value_type=None):
        if key in dict_ins:
            value = dict_ins.get(key)
            if value_type is not None:
                if isinstance(value, value_type) is False:
                    value = None
        else:
            value = None
        return value

    def json_loads(self, json_str):
        # print "load json:",json_str
        try:
            strings = json.loads(json_str, encoding='utf-8')
        except:
            print("Parsing json string error! json_str:", json_str)
            return
        else:
            return strings

    def json_dumps(self, dict_str):
        # print "dict:",dict_str
        try:
            # note [ensure_ascii=False] must be set, then utf-8 chinese code can be use
            json_str = json.dumps(dict_str, ensure_ascii=False)
        except:
            print("Dict to json error! dict:", dict_str)
            return
        else:
            return json_str

    def get_params(self, cmd):
        try:
            if 'params' in cmd:
                param_json = cmd.get('params')
        except:
            print("Cmd params error! cmd:", cmd)
            return
        else:
            param = self.json_loads(param_json)
            return param

    def push_msg_cond(self, msg_cond):
        self.msg_cond_list_lock_.acquire()
        self.msg_cond_list_.append(msg_cond)
        self.msg_cond_list_lock_.release()

    def delete_msg_cond(self, msg_cond):
        self.msg_cond_list_lock_.acquire()
        self.msg_cond_list_.remove(msg_cond)
        self.msg_cond_list_lock_.release()

    def get_seq(self):
        self.send_seq_lock_.acquire()
        seq = self.send_seq_
        self.send_seq_ = self.send_seq_ + 1
        self.send_seq_lock_.release()
        return seq

    def send_msg(self, cmd, params, wait=True):
        ret = 0

        if self.adapter_name_ is None or self.actuator_name_ is None:
            return -1, None

        timestamp_handle_start = int(time.time() * 1000)
        seq = self.get_seq()
        if params is None:
            params = {}
        params['seq'] = seq
        params_json = self.json_dumps(params)

        actuator_cmd = self.actuator_name_ + ':' + cmd

        msg_cond = MsgCondition(seq, self.adapter_name_, actuator_cmd)
        self.push_msg_cond(msg_cond)

        self.proxy_.send(self.adapter_name_, actuator_cmd, params_json)
        if wait is False:
            self.delete_msg_cond(msg_cond)
            return 0, None

        retry_cnt = 0
        while msg_cond.status == "init" and retry_cnt < 3:
            msg_cond.ack_event_.wait(MSG_ACK_TIMEOUT_MS/1000)
            if msg_cond.status == "init":
                self.proxy_.send(self.adapter_name_, actuator_cmd, params_json)
                retry_cnt = retry_cnt + 1
        if msg_cond.status == "ack":
            msg_cond.res_event_.wait(MSG_WAIT_INFINITE)
            ret = msg_cond.code

        if msg_cond.status != "res":
            ret = -1

        timestamp_handle_end = int(time.time() * 1000)
        # print("{}:cmd[{}] handle time:{}".format(self.name_, cmd,
        #                                         timestamp_handle_end - timestamp_handle_start))
        if msg_cond.result_data is not None:
            result_data = msg_cond.result_data
        else:
            result_data = None

        self.delete_msg_cond(msg_cond)

        return ret, result_data

    def subscribe(self, topic, callback):
        topic_list = []
        topic_name = "{0}:{1}:{2}".format(self.adapter_name_, self.actuator_name_, topic)
        topic_list.append(topic_name)
        self.topic_list_.append((topic_name, callback))
        self.topic_socket_.subscribe(topic_list)
        # print("{}:subscribe topic:{}".format(self.name_, topic_name))


class ArmInterface:
    def __init__(self, proxy_ip, proxy_name, target_name='arm', client_name='client', core_name='core_1'):
        self.name_ = target_name
        self.arm_proxy_ = ActuatorInterface(client_name, proxy_ip)
        self.arm_proxy_.set_target_direct(target_name, proxy_name)

        self.cur_pose = []
        self.cur_joint = []
        self.timestamp_data_refresh_ = 0.0
        self.arm_is_program_run_ = False
        self.data_lock_ = threading.Lock()

        self.arm_proxy_.subscribe("poseinfo", self.poseinfo_topic)
        self.start_topic()

    def get_pos(self):
        # print("{}:get current pos".format(self.name_))
        ret, pos = self.arm_proxy_.send_msg('getpos', None)
        return ret, pos

    def get_joint(self):
        # print("{}:get current joint".format(self.name_))
        ret, pos = self.arm_proxy_.send_msg('getjoint', None)
        return ret, pos

    def set_pose_curve(self, pose, acc=1.0, speed=1.0, time=0.0, wait=True):
        return self.point(pose, 'curve', acc=acc, spd=speed, time=time, wait=wait)

    def set_pose_line(self, pose, acc=0.5, speed=1.0, time=0.0, wait=True):
        return self.point(pose, 'line', acc=acc, spd=speed, time=time, wait=wait)

    def point(self, pos, mode="line", acc=0.1, spd=0.5, time=0, radius=0.01, wait=True):
        # print("{}:move to pos {}".format(self.name_, pos))
        params_dic = {
            'pos': pos,
            'mode': mode,
            'acc': acc,
            'spd': spd,
            'time': time,
            'radius': radius,
        }
        ret, result_data = self.arm_proxy_.send_msg('point', params_dic, wait=wait)
        return ret, result_data

    def joint(self, joint, acc=0.1, spd=0.5, time=0.0, wait=True):
        # print("{}:move to joint {}".format(self.name_, joint))
        params_dic = {
            'joint': joint,
            'time': time,
        }
        ret, result_data = self.arm_proxy_.send_msg('joint', params_dic, wait=wait)
        return ret, result_data
    '''
    def servojs(self, pose_list, time_interval=0.008, lookahead_time=0.1, gain=300, isjoint=True, wait=True):
        params_dic = {
            'poselist': pose_list,
            'time': time_interval,
            'isjoint': isjoint,
            'lookahead': lookahead_time,
            'gain': gain,
        }
        ret, relult_data = self.arm_proxy_.send_msg('servojs', params_dic, wait=wait)
        return ret
    '''
    def servojs(self, pose_list, time_interval, lookahead_time_list, gain_list, isjoint=True, wait=True):
        params_dic = {
            'poselist': pose_list,
            'time': time_interval,
            'isjoint': isjoint,
            'lookahead': lookahead_time_list,
            'gain': gain_list,
        }
        ret, relult_data = self.arm_proxy_.send_msg('servojs', params_dic, wait=wait)
        return ret

    def movels(self, pose_list, acc=0.5, speed=1.0, time=0.0, blend_radius=0.01, wait=True):
        params_dic = {
            'poselist': pose_list,
            'acc': acc,
            'spd': speed,
            'time': time,
            'radius': blend_radius
        }
        ret, relult_data = self.arm_proxy_.send_msg('movelist', params_dic, wait=wait)
        return ret

    def play_record(self, path, step=0):
        params_dic = {
            'mode': 'play',
            'path': path,
            'step': step,
        }
        if step == 0:
            waits = True
        else:
            waits = False
        ret, relult_data = self.arm_proxy_.send_msg('record', params_dic, wait=waits)
        return ret

    def start_record(self, path):
        params_dic = {
            'mode': 'start',
            'path': path,
        }
        ret, relult_data = self.arm_proxy_.send_msg('record', params_dic, wait=False)
        return ret

    def stop_record(self):
        params_dic = {
            'mode': 'stop',
        }
        ret, relult_data = self.arm_proxy_.send_msg('record', params_dic, wait=False)
        return ret

    def rt_play_start(self, sample):
        params_dic = {
            'cmd': 'start',
            'sample': sample,
        }
        ret, relult_data = self.arm_proxy_.send_msg('playrealtime', params_dic)
        return ret

    def rt_play_end(self):
        params_dic = {
            'cmd': 'end',
        }
        ret, relult_data = self.arm_proxy_.send_msg('playrealtime', params_dic)
        return ret

    def rt_play_data(self, pose):
        params_dic = {
            'cmd': 'data',
            'pose': pose,
        }
        ret, relult_data = self.arm_proxy_.send_msg('playrealtime', params_dic)
        return ret

    def circle(self, radius, time, repeat):
        params_dic = {
            'radius': radius,
            'time': time,
            'repeat': repeat
        }
        ret, relult_data = self.arm_proxy_.send_msg('circle', params_dic)
        return ret

    def poseinfo_topic(self, msg):

        #print("{}:get poseinfo topic:{}".format(self.name_, msg))
        poseinfo_dic = self.json_loads(msg)

        if poseinfo_dic is not None:
            with self.data_lock_:
                pose = self.get_dict_key_value(poseinfo_dic, 'tcpdata', list)
                if pose is not None:
                    self.cur_pose = pose[:]
                joint = self.get_dict_key_value(poseinfo_dic, 'q_actual', list)
                if joint is not None:
                    self.cur_joint = joint[:]
                is_run = self.get_dict_key_value(poseinfo_dic, 'is_run', bool)
                if is_run is not None:
                    self.arm_is_program_run_ = is_run
                else:
                    print("error for parmas [is_run]")

                self.timestamp_data_refresh_ = time.time()


    def get_cur_pose(self, with_timestamp=False):
        ret = 0
        if self.timestamp_data_refresh_ + 0.100 < time.time():
            print('error! too long did not refresh data! refesh:{} cur:{}'.format(self.timestamp_data_refresh_, time.time()))
            return -1, None

        with self.data_lock_:
            pose = self.cur_pose[:]
            timestamp = self.timestamp_data_refresh_
        if with_timestamp is True:
            return ret, pose, timestamp
        else:
            return ret, pose

    def get_cur_joint(self, with_timestamp=False):
        ret = 0
        if self.timestamp_data_refresh_ + 0.100 < time.time():
            print('error! too long did not refresh data! refesh:{} cur:{}'.format(self.timestamp_data_refresh_, time.time()))
            return 0, None

        with self.data_lock_:
            joint = self.cur_joint[:]
            timestamp = self.timestamp_data_refresh_
        if with_timestamp is True:
            return ret, joint, timestamp
        else:
            return ret, joint

    def is_program_running(self):
        with self.data_lock_:
            is_run = self.arm_is_program_run_
        return is_run

    def start_topic(self):
        params_dic = {
            'topic': 'poseinfo',
        }
        ret, relult_data = self.arm_proxy_.send_msg('startpub', params_dic)
        return ret
    def close(self):
        self.proxy_.close()

    def inverse_kinematic(self, pose, joint_ref=None):
        params_dic = {
            'pos': pose
        }
        if joint_ref is not None:
            params_dic['jointref'] = joint_ref

        ret, joint = self.arm_proxy_.send_msg('inversekin', params_dic)
        return ret, joint

    def inverse_kinematic_list(self, pose):
        params_dic = {
            'pos_list': pose
        }
        ret, joint = self.arm_proxy_.send_msg('inverse_kin_list', params_dic)
        return ret, joint

    def set_tool_offset(self, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        params_dic = {
            'X': float(x),
            'Y': float(y),
            'Z': float(z),
            'Rx': float(rx),
            'Ry': float(ry),
            'Rz': float(rz),
        }

        ret, result_data = self.arm_proxy_.send_msg('settcpoffset', params_dic)

        return ret

    def get_tool_offset(self):
        ret, tool_offset = self.arm_proxy_.send_msg('gettcpoffset', None)
        return ret, tool_offset

    def start_teach_mode(self):
        ret = self.arm_proxy_.send_msg('startteach', None)
        return ret

    def stop_teach_mode(self):
        ret = self.arm_proxy_.send_msg('stopteach', None)
        return ret

    def get_robot_serial_number(self):
        ret, serial_number = self.arm_proxy_.send_msg('get_robot_serial_number', None)
        return ret, serial_number

    def move_process(self, pos, acc=0.1, spd=0.5, radius=0.01, wait=True):
        # print("{}:move to pos {}".format(self.name_, pos))
        params_dic = {
            'pos': pos,
            'acc': acc,
            'spd': spd,
            'radius': radius,
        }
        ret, result_data = self.arm_proxy_.send_msg('move_process', params_dic, wait=wait)
        return ret, result_data

    def json_loads(self, json_str):
        # print "load json:",json_str
        try:
            strings = json.loads(json_str, encoding='utf-8')
        except:
            print("Parsing json string error! json_str:", json_str)
            return
        else:
            return strings

    # value_type: int, float, str, bool, list, dict
    def get_dict_key_value(self, dict_ins, key, value_type=None):
        if key in dict_ins:
            value = dict_ins.get(key)
            if value_type is not None:
                if isinstance(value, value_type) is False:
                    value = None
        else:
            value = None
        return value

class SocketMsgCondition:
    def __init__(self, seq, cmd):
        self.seq_ = seq
        self.cmd_ = cmd
        self.condition_ = threading.Condition()
        self.status = "init"
        self.result_data = None
        self.is_has_handle = False
        self.code = 0
        self.codestring = ''


class SocketMsgInterface:
    def __init__(self, socket_name='lubancmd', socket_ip="192.168.1.102"):
        self.socket_name_ = socket_name
        self.socket_ip_ = socket_ip
        self.condition_ = threading.Condition()
        self.msg_cond_list_lock_ = threading.Lock()
        self.msg_cond_list_ = []
        self.send_seq_ = 0
        self.send_seq_lock_ = threading.Lock()

        self.proxy_socket_ = Socket("tcp://" + socket_ip + ":5000", socket_name.encode('ascii'), self, self.recv_callback)
        self.proxy_socket_.connect()

    def recv_callback(self, args, msg):
        #print(msg)

        cmd = self.get_dict_key_value(msg, 'cmd', bytes)
        cmd = str(cmd, encoding='utf-8')
        if cmd is None:
            print("cmd error")
            return

        params = self.get_params(msg)

        #params = str(params, encoding='utf-8')
        seq = self.get_dict_key_value(params, 'seq', int)

        if seq is None:
            # print("seq error")
            return

        type = self.get_dict_key_value(params, 'type', STRING_TYPE)

        if type is None:
            print("type error")
            return

        code = self.get_dict_key_value(params, 'code', int)
        if code is None:
            print("code error")
            return

        codestring = self.get_dict_key_value(params, 'errmsg', STRING_TYPE)
        if codestring is None:
            codestring = ''

        data = self.get_dict_key_value(params, 'data')

        self.msg_cond_list_lock_.acquire()
        for msg_cond in self.msg_cond_list_:
            #print(msg_cond.seq_, msg_cond.is_has_handle, type, msg_cond.cmd_)
            if msg_cond.seq_ == seq and type == 'res' \
                    and msg_cond.cmd_ == cmd \
                    and msg_cond.is_has_handle is False:
                # print("get cond")
                msg_cond.is_has_handle = True
                msg_cond.status = "res"
                msg_cond.code = code
                msg_cond.codestring = codestring
                with msg_cond.condition_:
                    msg_cond.condition_.notify()
            else:
                print("unknow replay type!")
        self.msg_cond_list_lock_.release()

    def sendAndWait(self, cmd, params):
        ret = 0

        seq = self.get_seq()
        if params is None:
            params = {}
        params['seq'] = seq
        params_json = self.json_dumps(params).encode('ascii')

        self.proxy_socket_.c_send(cmd.encode('ascii'), params_json)

        msg_cond = SocketMsgCondition(seq, cmd)
        self.push_msg_cond(msg_cond)
        # print("wait ack...")

        with msg_cond.condition_:
            msg_cond.condition_.wait(MSG_WAIT_INFINITE)

        if msg_cond.status != "res":
            ret = -1
        self.delete_msg_cond(msg_cond)

        #print("return {}".format(ret))

        return ret
    # value_type: int, float, str, bool, list, dict
    def get_dict_key_value(self, dict_ins, key, value_type=None):
        if key in dict_ins:
            value = dict_ins.get(key)
            if value_type is not None:
                if isinstance(value, value_type) is False:
                    value = None
        else:
            value = None
        return value

    def json_loads(self, json_str):
        # print "load json:",json_str
        try:
            strings = json.loads(json_str, encoding='utf-8')
        except:
            print("Parsing json string error! json_str:", json_str)
            return
        else:
            return strings

    def json_dumps(self, dict_str):
        # print "dict:",dict_str
        try:
            # note [ensure_ascii=False] must be set, then utf-8 chinese code can be use
            json_str = json.dumps(dict_str, ensure_ascii=True)
        except:
            print("Dict to json error! dict:", dict_str)
            return
        else:
            return json_str

    def get_params(self, cmd):
        try:
            if 'params' in cmd:
                param_json = cmd.get('params')
        except:
            print("Cmd params error! cmd:", cmd)
            return
        else:
            param = self.json_loads(param_json)
            return param

    def push_msg_cond(self, msg_cond):
        self.msg_cond_list_lock_.acquire()
        self.msg_cond_list_.append(msg_cond)
        self.msg_cond_list_lock_.release()

    def delete_msg_cond(self, msg_cond):
        self.msg_cond_list_lock_.acquire()
        self.msg_cond_list_.remove(msg_cond)
        self.msg_cond_list_lock_.release()

    def get_seq(self):
        self.send_seq_lock_.acquire()
        seq = self.send_seq_
        self.send_seq_ = self.send_seq_ + 1
        self.send_seq_lock_.release()
        return seq
    def cmd_make(self, project_name):
        make_dic = {
            'name': project_name,
            'varlist': [],
            'mac': '123',
            'uiStamp': 123,
            'wssrc': '123',
            'wsstamp': 123
        }
        # socket_inf.sendAndWait('make'.encode('ascii'), '{"seq":625,"name":"ttretret","varlist":[],"mac":"sdf","uiStamp":123,"wssrc":"123","wsstamp":123}'.encode('ascii'))
        ret = self.sendAndWait('make', make_dic)
        return ret


def main():



    proxy_ip = "169.254.53.102"

    arm_interface_1 = ArmInterface(proxy_ip, 'ag_urarm_1', 'arm', 'client1')

    acc = 1500
    speed = 1000
    count = 0
    while True:
        '''
        pose = arm_left.get_pose()
        count += 1
        # print("{}:count={}, pose={}".format(time.time(), count, pose))

        pose = arm_left.get_joints()
        # print("{}:count={}, joint={}".format(time.time(), count, pose))

        is_run = arm_left.is_program_running()
        # print("is program run:{}".format(is_run))
        '''
        time.sleep(1)


def exit_handle(signum, frame):
    print("You choose to stop me through ctrl+c.")
    sys.exit()


if __name__ == "__main__":

    # register exit function for exit by 'ctrl+c'
    signal.signal(signal.SIGINT, exit_handle)
    signal.signal(signal.SIGTERM, exit_handle)

    main()

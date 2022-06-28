#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import time
import json
import threading
from math import pi
from rlog import rlog
import numpy as np
log = rlog()

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/adapter")
sys.path.append(abs_file + "/../../../lib/log")

# print sys.path
from copy import deepcopy
from proxy_client import PS_Socket
from actuator import Actuator
from actuator import ErrorInfo
from actuator import ActuatorCmdType
from actuator import Status
from actuator import JsonUtils
from controller_interface import ControllerInterface

# Define Error code
MOD_ERR_NUM = 3900
MOD_ERR_SELF_OFFSET = 20
E_OK = 0
E_MOD_PARAM = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 1
E_MOD_STATUS = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 2
E_MOD_DRIVER = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 3
E_MOD_EXCEPTION = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 5
E_MOD_ABORT_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 6
E_MOD_EXEC_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 7

# command description dict
cmd_description_dict = {
            'cmddescribe': {
                'version': 'v0.0.1',
                'date': '20200623',
                'time': '16:06:25',
            },
            'cmdlist': [
                {
                    'cmd': 'grip',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'spose',
                            'type': 'array',
                            'default': [0,0,0,0,0,0],
                            'numberlimit': [-2*pi, 2*pi]
                        },
                        {
                            'name': 'epose',
                            'type': 'array',
                            'default': [0,0,0,0,0,0],
                            'numberlimit': [-2*pi, 2*pi]
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'eefstep',
                            'type': 'float',
                            'default':  0.001,
                            'numberlimit': [0.0001, 0.1]
                        },
                        {
                            'name': 'grippos',
                            'type': 'int',
                            'default': 10,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        },
                        {
                            'name': 'gripspeed',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        },
                        {
                            'name': 'gripeffort',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        }
                    ]
                },
                {
                    'cmd': 'move',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'spose',
                            'type': 'array',
                            'default':  [0,0,0,0,0,0],
                            'numberlimit': [-2*pi, 2*pi]
                        },
                        {
                            'name': 'epose',
                            'type': 'array',
                            'default':  [0,0,0,0,0,0],
                            'numberlimit': [-2*pi, 2*pi]
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'eefstep',
                            'type': 'float',
                            'default':  0.001,
                            'numberlimit': [0.0001, 0.1]
                        }
                    ]
                },
                {
                    'cmd': 'release',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'spose',
                            'type': 'array',
                            'default':  [0,0,0,0,0,0],
                            'numberlimit': [-2*pi, 2*pi]
                        },
                        {
                            'name': 'epose',
                            'type': 'array',
                            'default':  [0,0,0,0,0,0],
                            'numberlimit': [-2*pi, 2*pi]
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'eefstep',
                            'type': 'float',
                            'default':  0.001,
                            'numberlimit': [0.0001, 0.1]
                        },
                        {
                            'name': 'grippos',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        },
                        {
                            'name': 'gripspeed',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        },
                        {
                            'name': 'gripeffort',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        }
                    ]
                },
                {
                    'cmd': 'gripper_control',
                    'atype': 'motion',
                    'params': [

                        {
                            'name': 'grippos',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        },
                        {
                            'name': 'gripspeed',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        },
                        {
                            'name': 'gripeffort',
                            'type': 'int',
                            'default': 50,
                            'numberlimit': [0, 100],
                            'unit': '%'
                        }
                    ]
                },
                {
                    'cmd': 'turn_side',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'joints',
                            'type': 'object',
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                    ]
                },
                {
                    'cmd': 'long_plan',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'tarpose',
                            'type': 'object',
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'eefstep',
                            'type': 'float',
                            'default':  0.001,
                            'numberlimit': [0.0001, 0.1]
                        }
                    ]
                },
                {
                    'cmd': 'mid_plan',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'tarpose',
                            'type': 'object',
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'eefstep',
                            'type': 'float',
                            'default':  0.001,
                            'numberlimit': [0.0001, 0.1]
                        }
                    ]
                },
                {
                    'cmd': 'short_plan',
                    'atype': 'motion',
                    'params': [
                        {
                            'name': 'tarpose',
                            'type': 'object',
                        },
                        {
                            'name': 'moveacc',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'movespeed',
                            'type': 'float',
                            'default': 0.5,
                            'numberlimit': [0, 2]
                        },
                        {
                            'name': 'eefstep',
                            'type': 'float',
                            'default':  0.001,
                            'numberlimit': [0.0001, 0.1]
                        }
                    ]
                },
                {
                    'cmd': 'set_euler_pos',
                    'atype': 'sensing',
                    'params':[
                      	{
                            'name': 'x',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-1.0,1.0],
                            'unit': 'mm'
                        }, 
                        {
                            'name': 'y',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-1.0,1.0],
                            'unit': 'mm'
                        },
                        {
                            'name': 'z',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-1.0,1.0],
                            'unit': 'mm'
                        },
                        {
                            'name': 'rx',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                        {
                            'name': 'ry',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                        {
                            'name': 'rz',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                    ],
                    'return':{
                        'type': 'object'
                    }
                },
                {
                    'cmd': 'set_joints',
                    'atype': 'sensing',
                    'params':[
                      	{
                            'name': 'joint6',
                            'type': 'float',
                            'default': 0,
                             'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        }, 
                        {
                            'name': 'joint5',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                        {
                            'name': 'joint4',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                        {
                            'name': 'joint3',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                        {
                            'name': 'joint2',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                        {
                            'name': 'joint1',
                            'type': 'float',
                            'default': 0,
                            'numberlimit': [-360.0, 360.0],
                            'unit': '°C'
                        },
                    ],
                    'return':{
                        'type': 'object'
                    }
                }
            ]
        }


class ActuatorMoveit(Actuator):
    def __init__(self, name, is_simulation, proxy_name, proxy_ip):
        Actuator.__init__(self, name)
        self.is_simulation_ = is_simulation
        self.data_condition_ = threading.Condition()
        self.is_async_working = False
        self.status = Status.uninit
        self.error_code_ = 0
        self.enable_timer = True
        self.proxy_name_ = proxy_name
        self.proxy_ip = proxy_ip
        log.notice("ActuatorMoveit start, is_simulation:{}".format(is_simulation))

    def connect(self):
        self.controller_ = ControllerInterface(proxy_ip=self.proxy_ip, adapter_name='ag_urarm_1')
        local_test = False
        if local_test and not self.is_simulation:
            spose = [-0.548, 0.322, -0.021, -3.122, -0.768, 2.908]
            mpose = [-0.653,0.381,-0.031,-3.103,-1.046,2.552]
            epose = [-0.122,0.482,0.271,-3.135,-0.730,1.510]
            moveacc = 0.1
            movespeed = 0.1
            grippos = 0.1
            gripspeed = 0.1
            gripeffort = 50
            self.controller_.set_grip_beef(spose, mpose, epose, moveacc, movespeed, grippos, gripspeed, gripeffort)
        self.set_status(Status.idle)

    def get_status_dict(self):
        status_dic = {
            'status': self.get_status(),
            'code': self.get_error_code(),
            'is_simulation': self.is_simulation_,
        }
        return status_dic

    # override function
    def sync_cmd_handle(self, msg):
        log.notice("sync_cmd_handle:{}".format(msg.cmd))
        is_has_handle = True
        value_ret = None
        error_code = 0
        if msg.cmd == "getcmdlist":
            value_ret = self.get_cmd_list()
        elif msg.cmd == "getstatus":
            value_ret = self.get_status_dict()
        elif msg.cmd == "set_euler_pos":
            x_ = JsonUtils.get_dict_key_value(msg.params, 'x', (float, int))
            y_ = JsonUtils.get_dict_key_value(msg.params, 'y', (float, int))
            z_ = JsonUtils.get_dict_key_value(msg.params, 'z', (float, int))
            rx_ = JsonUtils.get_dict_key_value(msg.params, 'rx', (float, int))
            ry_ = JsonUtils.get_dict_key_value(msg.params, 'ry', (float, int))
            rz_ = JsonUtils.get_dict_key_value(msg.params, 'rz', (float, int))
            if x_ is not None and y_ is not None and z_ is not None and\
                    rx_ is not None and ry_ is not None and rz_ is not None:
                if x_ != 0 or y_ != 0 or z_ != 0 or rx_ != 0 or ry_ != 0 or rz_ != 0:  
                    value_ret = {'x':x_,'y':y_,'z':z_,'roll':rx_,'pitch':ry_,'yaw':rz_}
                else:
                    error_code = E_MOD_PARAM
            else:
                error_code = E_MOD_PARAM
        elif msg.cmd == "set_joints":
            joint6_ = JsonUtils.get_dict_key_value(msg.params, 'joint6', (float, int))
            joint5_ = JsonUtils.get_dict_key_value(msg.params, 'joint5', (float, int))
            joint4_ = JsonUtils.get_dict_key_value(msg.params, 'joint4', (float, int))
            joint3_ = JsonUtils.get_dict_key_value(msg.params, 'joint3', (float, int))
            joint2_ = JsonUtils.get_dict_key_value(msg.params, 'joint2', (float, int))
            joint1_ = JsonUtils.get_dict_key_value(msg.params, 'joint1', (float, int))
            if joint6_ is not None and joint5_ is not None and joint4_ is not None and\
                    joint3_ is not None and joint2_ is not None and joint1_ is not None:
                if joint6_ != 0 or joint5_ != 0 or joint4_ != 0 or joint3_ != 0 or joint2_ != 0 or joint1_ != 0:  
                    value_ret = {'joint6':joint6_,'joint5':joint5_,'joint4':joint4_,'joint3':joint3_,'joint2':joint2_,'joint1':joint1_}
                else:
                    error_code = E_MOD_PARAM
            else:
                error_code = E_MOD_PARAM
        else:
            is_has_handle = False

        if True == is_has_handle:
            if 0 == error_code:
                error_info = ErrorInfo(error_code, "")
            elif E_MOD_PARAM == error_code:
                error_info = ErrorInfo(error_code, "params error!")
            else:
                error_info = ErrorInfo(error_code, "execution error!")
            self.reply_result(msg, error_info, value_ret)
        return is_has_handle

    # override function
    def async_cmd_handle(self, msg):
        error_code = 0
        error_info = ErrorInfo(0, "")
        is_has_handle = True
        value_ret = None
        self.set_async_status(True)
        log.notice("async_cmd_handle cmd:{}, params:{}".format(msg.cmd, msg.params))
        moveacc_ = JsonUtils.get_dict_key_value(msg.params, 'moveacc', (float, int))
        movespeed_ = JsonUtils.get_dict_key_value(msg.params, 'movespeed', (float, int))
        eefstep_ = JsonUtils.get_dict_key_value(msg.params, 'eefstep', (float, int))
        if msg.cmd == "short_plan":
            tarpose_ = JsonUtils.get_dict_key_value(msg.params, 'tarpose', list)
            log.notice("do short_plan, params-> tarpose:{} acc:{} mspeed:{} step:{}".format(tarpose_, moveacc_, movespeed_, eefstep_))
            if tarpose_ is not None and moveacc_ is not None and movespeed_ is not None and eefstep_ is not None:
                ret = self.controller_.short_dist_plan(targetpos=tarpose_, targetpos_list=None, joints=None, speed=movespeed_, acc=moveacc_, eef_step=eefstep_, end_effector_link="tool_pen")
                if ret is False:
                    error_code = E_MOD_EXEC_FAILED
                    error_info = ErrorInfo(error_code, "execute failed")
            else:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params invalid")
        elif msg.cmd == "mid_plan":
            tarpose_ = JsonUtils.get_dict_key_value(msg.params, 'tarpose', list)
            log.notice("do mid_plan, params-> tarpose:{} acc:{} mspeed:{} step:{}".format(tarpose_, moveacc_, movespeed_, eefstep_))
            if tarpose_ is not None and moveacc_ is not None and movespeed_ is not None and eefstep_ is not None:
                ret = self.controller_.mid_dist_plan(targetpos=tarpose_, joints=None, speed=movespeed_, acc=moveacc_, eef_step=eefstep_, end_effector_link="tool_pen")
                if ret is False:
                    error_code = E_MOD_EXEC_FAILED
                    error_info = ErrorInfo(error_code, "execute failed")
            else:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params invalid")
        elif msg.cmd == "long_plan":
            tarpose_ = JsonUtils.get_dict_key_value(msg.params, 'tarpose', list)
            log.notice("do long_plan, params-> tarpose:{} acc:{} mspeed:{} step:{}".format(tarpose_, moveacc_, movespeed_, eefstep_))
            if tarpose_ is not None and moveacc_ is not None and movespeed_ is not None and eefstep_ is not None:
                ret = self.controller_.long_dist_plan(targetpos=tarpose_, joints=None, speed=movespeed_, acc=moveacc_, eef_step=eefstep_, end_effector_link="tool_pen")
                if ret is False:
                    error_code = E_MOD_EXEC_FAILED
                    error_info = ErrorInfo(error_code, "execute failed")
            else:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params invalid")
        elif msg.cmd == "move":
            spose_ = JsonUtils.get_dict_key_value(msg.params, 'spose', list)
            epose_ = JsonUtils.get_dict_key_value(msg.params, 'epose', list)
            log.notice("do move, params-> spose:{} epose:{} acc:{} mspeed:{} step:{}".format(spose_, epose_, moveacc_, movespeed_, eefstep_))
            if spose_ is not None and epose_ is not None and moveacc_ is not None and movespeed_ is not None and eefstep_ is not None:
                ret = self.controller_.set_move_beef(spose=spose_, epose=epose_, speed=movespeed_, acc=moveacc_, eef_step=eefstep_, end_effector_link="tool_pen")
                if ret is False:
                    error_code = E_MOD_EXEC_FAILED
                    error_info = ErrorInfo(error_code, "execute failed")
            else:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params invalid")
        elif msg.cmd == "grip":
            spose_ = JsonUtils.get_dict_key_value(msg.params, 'spose', list)
            epose_ = JsonUtils.get_dict_key_value(msg.params, 'epose', list)
            grippos_ = JsonUtils.get_dict_key_value(msg.params, 'grippos', int)
            gripspeed_ = JsonUtils.get_dict_key_value(msg.params, 'gripspeed', int)
            gripeffort_ = JsonUtils.get_dict_key_value(msg.params, 'force', int)
            log.notice("do grip, params-> spose:{} epose:{} grip:{} gspeed:{} effort:{} acc:{} mspeed:{} step:{}".format(spose_, epose_, grippos_, gripspeed_, gripeffort_, moveacc_, movespeed_, eefstep_))
            if spose_ is not None and epose_ is not None \
                    and grippos_ is not None and gripspeed_ is not None and gripeffort_ is not None \
                    and moveacc_ is not None and movespeed_ is not None and eefstep_ is not None:
                ret = self.controller_.set_grip_beef(spose=spose_, epose=epose_, grippos=grippos_, gripspeed=gripspeed_, gripeffort=gripeffort_, speed=movespeed_, acc=moveacc_, eef_step=eefstep_, end_effector_link="tool_pen")
                if ret is False:
                    error_code = E_MOD_EXEC_FAILED
                    error_info = ErrorInfo(error_code, "execute failed")
            else:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params invalid")
        elif msg.cmd == "release":
            spose_ = JsonUtils.get_dict_key_value(msg.params, 'spose', list)
            epose_ = JsonUtils.get_dict_key_value(msg.params, 'epose', list)
            grippos_ = JsonUtils.get_dict_key_value(msg.params, 'grippos', int)
            gripspeed_ = JsonUtils.get_dict_key_value(msg.params, 'gripspeed', int)
            gripeffort_ = JsonUtils.get_dict_key_value(msg.params, 'force', int)
            log.notice("do release, params-> spose:{} epose:{} grip:{} gspeed:{} effort:{} acc:{} mspeed:{} step:{}".format(spose_, epose_, grippos_, gripspeed_, gripeffort_, moveacc_, movespeed_, eefstep_))
            if spose_ is not None and epose_ is not None \
                    and grippos_ is not None and gripspeed_ is not None and gripeffort_ is not None \
                    and moveacc_ is not None and movespeed_ is not None and eefstep_ is not None:
                ret = self.controller_.set_release_beef(spose=spose_, epose=epose_, grippos=grippos_, gripspeed=gripspeed_, gripeffort=gripeffort_, speed=movespeed_, acc=moveacc_, eef_step=eefstep_, end_effector_link="tool_pen")
                if ret is False:
                    error_code = E_MOD_EXEC_FAILED
                    error_info = ErrorInfo(error_code, "execute failed")
            else:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params invalid")
        elif msg.cmd == "turn_side":
            joints_obj_ = JsonUtils.get_dict_key_value(msg.params, 'joints', dict)
            joints_ = None
            if joints_obj_ is not None:
                joints_ = []
                joints_.append(joints_obj_['joint6']/180.0*pi)
                joints_.append(joints_obj_['joint5']/180.0*pi)
                joints_.append(joints_obj_['joint4']/180.0*pi)
                joints_.append(joints_obj_['joint3']/180.0*pi)
                joints_.append(joints_obj_['joint2']/180.0*pi)
                joints_.append(joints_obj_['joint1']/180.0*pi)
            log.notice("do turn_side, params-> joints:{} movespeed:{} moveacc:{}".format(joints_,movespeed_, moveacc_))
            ret = self.controller_.fk_turn_plan(joints = joints_ ,speed=movespeed_, acc=moveacc_)
            if ret is False:
                error_code = E_MOD_EXEC_FAILED
                error_info = ErrorInfo(error_code, "execute failed")
        else:
            is_has_handle = False
            log.error("invalid command:{}".format(msg.cmd))

        if True == is_has_handle:
            if 0 == error_code:
                error_info = ErrorInfo(error_code, "")
            elif E_MOD_PARAM == error_code:
                error_info = ErrorInfo(error_code, "params error!")
            else:
                error_info = ErrorInfo(error_code, "execution error!")
            self.reply_result(msg, error_info, value_ret)

        self.set_async_status(False)
        return is_has_handle

    # override function
    def abort_handle(self):
        log.warn("{}: abort_handle".format(self.name_))

    # override function
    def reset_handle(self):
        log.warn("{}: reset_handle".format(self.name_))
        return True

    def get_cmd_list(self):
        return cmd_description_dict

    def set_status(self, status):
        with self.data_condition_:
            self.status = status

    def get_status(self):
        with self.data_condition_:
            status = self.status
        return status

    def set_error_code(self, error_code):
        with self.data_condition_:
            self.error_code_ = error_code

    def get_error_code(self):
        with self.data_condition_:
            error_code = self.error_code_
        return error_code

    def set_async_status(self, is_working):
        with self.data_condition_:
            self.is_async_working = is_working

    def get_async_status(self):
        with self.data_condition_:
            is_working = self.is_async_working
        return is_working

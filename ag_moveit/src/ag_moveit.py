#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import time
import signal
import json

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/adapter")
sys.path.append(abs_file + "/../../../lib/log")
# print sys.path

from actuator import Adapter
from actuator import Status
from actuator import JsonUtils
from actuator_moveit import ActuatorMoveit
from rlog import rlog
log = rlog()

ag_name = 'ag_moveit'
actuator_1_name = 'moveit'

def exit_handle(signum, frame):
    log.warn('You choose to stop me.')
    sys.exit()

if __name__ == "__main__":
    log.setModuleName(ag_name)
    log.notice("ag_moveit is running...")
    str_proxy_name = None
    str_sim = None
    is_simulation_moveit = False
    proxy_ip = '127.0.0.1'
    arm_ip_address = None

    # argv params parse
    argv_num = len(sys.argv)
    argv_index = 1
    while argv_index < argv_num:
        if '--proxy' == sys.argv[argv_index]:
            if argv_index + 1 < argv_num:
                if sys.argv[argv_index + 1].find('--') != 0:
                    log.notice("--proxy:{}".format(sys.argv[argv_index + 1]))
                    str_proxy_name = sys.argv[argv_index + 1]
                    argv_index = argv_index + 1
        elif '--sim' == sys.argv[argv_index]:
            if argv_index + 1 < argv_num:
                if sys.argv[argv_index + 1].find('--') != 0:
                    log.notice("--sim:{}".format(sys.argv[argv_index + 1]))
                    str_sim = sys.argv[argv_index + 1]
                    argv_index = argv_index + 1
        else:
            log.error("unknow params:{}".format(sys.argv[argv_index]))
        argv_index = argv_index + 1

    proxy_name = None
    # parse from config file
    unit_dict = JsonUtils.get_config_unit('application', 'actuator', ag_name)
    if unit_dict is not None:
        proxy_name = JsonUtils.get_dict_key_value(unit_dict, 'proxy', (str, unicode))
        log.notice("get proxy name({}) from config file".format(proxy_name))

    # proxy name rename
    if str_proxy_name is not None:
        log.warn("proxy name change to: {}".format(str_proxy_name))
        proxy_name = str_proxy_name

    if proxy_name is None:
        log.error("{} proxy name is None, can't start program!!!".format(ag_name))
        sys.exit()

    # simulation parse
    if str_sim is not None:
        log.info("str_sim is {}".format(str_sim))
        str_sim_list = str_sim.split(':')
        for sim_name in str_sim_list:
            if actuator_1_name == sim_name:
                log.notice("{} will run in simulation mode!".format(sim_name))
                is_simulation_moveit = True
            if 'all' == sim_name:
                log.notice("All ag_moveit will run in simulation mode!")
                is_simulation_moveit = True
                break

    # register exit function for exit by 'ctrl+c'
    signal.signal(signal.SIGINT, exit_handle)
    signal.signal(signal.SIGTERM, exit_handle)

    # create adapter and special actuator
    adapter = Adapter(proxy_name.encode('utf-8'))
    unit_dict = JsonUtils.get_config_unit('application', 'proxy', 'proxy')
    if unit_dict is None:
        proxy_ip = "localhost"
    else:
        proxy_ip = JsonUtils.get_dict_key_value(unit_dict, 'ip', (str, unicode))
        log.notice("get proxy ip({}) from config file".format(proxy_ip))

    # create adapter and special actuator
    actuator_moveit = ActuatorMoveit(name = actuator_1_name, is_simulation=is_simulation_moveit,  proxy_name = proxy_name, proxy_ip = proxy_ip)

    # register actuator to adapter
    adapter.reg_actuator(actuator_moveit)
    actuator_moveit.connect()
    adapter.set_status(Status.idle)
    log.notice("set status to ({})".format(Status.idle))

    while True:
        time.sleep(1)

#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：PythonProject 
@File    ：create_vehicles.py
@IDE     ：PyCharm 
@Author  ：Lu
@Date    ：2023/4/10 21:36 
'''

## Create HV、AV and Platoon

## Initialization module

import os
import sys

import plexe
import random
# 调用 utils module，里面包含了 platooning 的中层实现函数
from utils import add_platooning_vehicle, start_sumo, running, communicate, add_hv_vehicle
# 确保路径设置正确，python 能够搜索到 traci module
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
# 调用 plexe module, 包含了 platooning 的底层实现函数
from plexe import Plexe, ACC, CACC, RPM, GEAR, RADAR_DISTANCE
from ccparams import LENGTH, DISTANCE, SPEED

## Hyperparameter module



## Function module

# add a platoon of n vehicles
def create_platoon(plexe, pid, lane, n, real_engine=False):
    """
    Adds a set of platoons of n vehicles each to the simulation
    :param plexe: API instance
    :param pid: the id of vehicle
    :param lane: the lane id of vehicle
    :param n: number of vehicles of the platoon
    :param real_engine: set to true to use the realistic engine model,
    false to use a first order lag model
    :return: returns the topology of the platoon, i.e., a dictionary which
    indicates, for each vehicle, who is its leader and who is its front
    vehicle. The topology can the be used by the data exchange logic to
    automatically fetch data from leading and front vehicle to feed the CACC
    """
    # topology 中以 dictionary 的形式存储车辆之间的通信拓扑结构，包括每辆车的前车和头车编号
    topology = {}
    leader_id = "p.%d.0" % pid
    p_length = n * LENGTH + (n - 1) * DISTANCE
    for i in range(n):
        vid = "p.%d.%d" % (pid, i)
        # 调用 utils module 中的函数，将车辆编排成 platoon 的形式
        pos = (n - i + 1) * (DISTANCE + LENGTH) + 50.0
        # print(position)

        add_platooning_vehicle(plexe, vid, pos, lane, SPEED, DISTANCE, vtype="vtypeauto", real_engine=real_engine)

        # 将车辆保持在 lane 0 ，并忽略安全距离限制
        plexe.set_fixed_lane(vid, lane, False)

        # 设置车辆速度模式。车辆的速度有 5 个影响因素
        # https://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#speed_mode_.280xb3.29
        # 1\. safe speed
        # 2\. max accel
        # 3\. max speed
        # 4\. right of way at intersections
        # 5\. brake hard to avoid passing red light
        # 如果全都不考虑，则设置为 [0, 0, 0, 0, 0] = 0, 此时，车速完全由 traci 控制
        # 如果全都考虑，则设置为 [1, 1, 1, 1, 1] = 31
        # 如果只考虑 safe speed，则设置为 [0, 0, 0, 0, 1] = 1
        traci.vehicle.setSpeedMode(vid, 0)

        # 在 platoon 中头车采用 adaptive cruise control 的控制方式
        # 后边的跟车采用 cooperative adative cruise control 的控制方式
        plexe.use_controller_acceleration(vid, False)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
        else:
            plexe.set_active_controller(vid, CACC)

        if i > 0:
            topology[vid] = {"front": "p.%d.%d" % (pid, i - 1), "leader": "p.%d.0" % pid}
        else:
            topology[vid] = {}
    return topology, leader_id

def create_av(plexe, vid, pos, lane, real_engine=False):
    vid = "v.%d.%d" % (0, vid)
    add_platooning_vehicle(plexe, vid, pos, lane, SPEED, DISTANCE, vtype="vtypeauto", real_engine=real_engine)

    traci.vehicle.setSpeedMode(vid,0)
    plexe.use_controller_acceleration(vid, False)
    # plexe.set_fixed_lane(vid, lane, False)
    plexe.set_active_controller(vid, ACC)

    traci.vehicle.setColor(vid,(51, 153, 255))
    return  vid

def create_hv(plexe, vid, pos, lane, real_engine=False):
    vid = "v.%d.%d" % (1, vid)

    # random for HV type
    if random.random() >= 0.5:
        vtype = "passenger"
    else:
        vtype = "passenger2"

    add_hv_vehicle(plexe, vid, pos, lane, SPEED, DISTANCE, vtype=vtype, real_engine=real_engine)



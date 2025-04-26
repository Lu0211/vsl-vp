#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：Test230410.py 
@File    ：join.py
@Author  ：Lu
@Date    ：2023/4/12 10:16 
'''
import os
import sys

import plexe
import random
# 调用 utils module，里面包含了 platooning 的中层实现函数
from utils import add_platooning_vehicle, start_sumo, running, communicate, add_hv_vehicle, get_distance

# 确保路径设置正确，python 能够搜索到 traci module
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
# 调用 plexe module, 包含了 platooning 的底层实现函数
from create_vehicles import DISTANCE, SPEED
from plexe import RADAR_DISTANCE, Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION
from ccparams import JOIN_DISTANCE, GOING_TO_POSITION, OPENING_GAP, COMPLETED, SPEED





def get_in_position(plexe, jid, fid, LEADER, topology):
    """
    Makes the joining vehicle get close to the join position. This is done by
    changing the topology and setting the leader and the front vehicle for
    the joiner. In addition, we increase the cruising speed and set the desire distance between front car
    thee we switch to the "fake" CACC, which uses a given GPS distance instead of the radar
    distance to compute the control action
    :param plexe: API instance
    :param jid: id of the joiner
    :param fid: id of the vehicle that will become the predecessor of the joiner
    :param topology: the current platoon topology
    :return: the modified topology
    """
    # 先将拓扑结构导入
    topology[jid] = {"leader": LEADER, "front": fid}
    # 设置好速度
    plexe.set_cc_desired_speed(jid, SPEED + 15)
    # 设置好追赶 front 前车的车头时距
    plexe.set_path_cacc_parameters(jid, JOIN_DISTANCE)
    # 设置控制加速器为 FAKED_CACC ，用作追赶前方车辆
    plexe.set_active_controller(jid, FAKED_CACC)
    return topology


def open_gap(plexe, vid, jid, topology, n):
    """
    Makes the vehicle that will be behind the joiner open a gap to let the
    joiner in. This is done by creating a temporary platoon, i.e., setting
    the leader of all vehicles behind to the one that opens the gap and then
    setting the front vehicle of the latter to be the joiner. To properly
    open the gap, the vehicle leaving space switches to the "fake" CACC,
    to consider the GPS distance to the joiner
    :param plexe: API instance
    :param vid: vehicle that should open the gap
    :param jid: id of the joiner
    :param topology: the current platoon topology
    :param n: total number of vehicles currently in the platoon
    :return: the modified topology
    """
    # 找到加入车排位置的后面那辆车，将该车视作新车排的leader
    index = int(vid.split(".")[2])
    for i in range(index + 1, n):
        # temporarily change the leader
        topology["p.0.%d" % i]["leader"] = vid
    # 在将新leader的front前车设置为要加入老车排的新车辆，同时设置好新的加速度控制器和车头时距
    # the front vehicle if the vehicle opening the gap is the joiner
    topology[vid]["front"] = jid
    plexe.set_active_controller(vid, FAKED_CACC)
    plexe.set_path_cacc_parameters(vid, distance=JOIN_DISTANCE)
    return topology


def reset_leader(vid, topology, LEADER, n):
    """
    After the maneuver is completed, the vehicles behind the one that opened
    the gap, reset the leader to the initial one
    :param vid: id of the vehicle that let the joiner in
    :param topology: the current platoon topology
    :param n: total number of vehicles in the platoon (before the joiner)
    :return: the modified topology
    """
    index = int(vid.split(".")[2])
    for i in range(index + 1, n):
        # restore the real leader
        topology["p.0.%d" % i]["leader"] = LEADER
    return topology


def Join_platoon(plexe, pid, jid, Jpos, step, state, topology, N_v):
    """
    Add the AV to join the platoon
    :param plexe: API instance
    :param pid: the id of the joined platoon
    :param jid: the id of joiner AV 'v.0.%d'
    :param Jpos: the pos of the join_position in the platoon
    :param step: the step of simulation
    :param state: the joiner state, 0 for going to the position, 1 for opening the gap
    2 for joining the platoon
    :param topology: the topology of the platoon
    :param N_v: the number of platoon vehicles
    :return: returns the topology of the platoon and state, i.e., a dictionary which
    indicates, for each vehicle, who is its leader and who is its front
    vehicle. The topology can the be used by the data exchange logic to
    automatically fetch data from leading and front vehicle to feed the CACC
    """
    F_jid = "p.0.%d" % (Jpos - 1)
    B_jid = "p.0.%d" % Jpos
    LEADER = "p.%d.0" % pid
    if step == 100:
        # at 1 second, let the joiner get closer to the platoon
        topology = get_in_position(plexe, jid, F_jid, LEADER, topology)
    if state == GOING_TO_POSITION and step > 0:
        # when the distance of the joiner is small enough, let the others
        # open a gap to let the joiner enter the platoon
        if get_distance(plexe, jid, F_jid) < JOIN_DISTANCE + 1:
            state = OPENING_GAP
            topology = open_gap(plexe, B_jid, jid, topology, N_v)
    if state == OPENING_GAP:
        # when the gap is large enough, complete the maneuver
        if get_distance(plexe, B_jid, F_jid) > \
                2 * JOIN_DISTANCE + 2:
            state = COMPLETED
            plexe.set_fixed_lane(jid, 0, safe=False)
            plexe.set_active_controller(jid, CACC)
            plexe.set_path_cacc_parameters(jid, distance=DISTANCE)
            plexe.set_active_controller(B_jid, CACC)
            plexe.set_path_cacc_parameters(B_jid, distance=DISTANCE)
            topology = reset_leader(B_jid, topology, LEADER, N_v)
    return topology, state

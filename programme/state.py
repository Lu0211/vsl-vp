#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：PythonProject 
@File    ：state.py
@IDE     ：PyCharm 
@Author  ：Lu
@Date    ：2023/3/24 19:10 
'''

# Vehicle data
# index: position in the platoon (0 = first)
# speed: vehicle speed
# acceleration: vehicle acceleration
# positionX: position of the vehicle in the simulation
# positionY: position of the vehicle in the simulation
# time: time at which such information was read from vehicle's sensors
# length: vehicle length
# u: controller acceleration
# speedX: vehicle speed on the X axis
# speedY: vehicle speed on the Y axis
# angle: vehicle angle in radians


import os
import sys

import traci

import plexe
from tabulate import tabulate
from plexe import RADAR_DISTANCE, Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION

# 确保路径设置正确，python 能够搜索到 traci module
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# plexe.get_vehicle_data()

# 状态设置？
# 包含三部分，车排信息，AV车辆信息以及瓶颈信息
# 车排信息：排长的速度、加速度、位置
# AV车辆信息：速度、加速度、位置
# 瓶颈信息：所在车道id、位置


class State():
    def __init__(self, plexe, leader_vid, av_vid, Accident_vid):
        # leader information
        leader_data = plexe.get_vehicle_data(leader_vid)
        self.leader_speed = leader_data.speed
        self.leader_acceleration = leader_data.acceleration
        self.leader_pos = traci.vehicle.getPosition(leader_vid)

        # AV information
        av_data = plexe.get_vehicle_data(av_vid)
        self.av_speed = av_data.speed
        self.av_acceleration = av_data.acceleration
        self.av_pos = traci.vehicle.getPosition(av_vid)


        # Accident information
        self.accident_lane_id = traci.vehicle.getLaneIndex(Accident_vid)
        self.accident_pos = traci.vehicle.getPosition(Accident_vid)

    def get_state(self):
        data = [[self.leader_speed, self.leader_acceleration, self.leader_pos, self.av_speed, \
                self.av_acceleration, self.av_pos, self.accident_pos, self.accident_lane_id]]
        print(tabulate(data, headers=['Leader speed', 'Leader acc', 'Leader pos', 'Av spped', \
                                      'Av acc', 'Av pos', 'Accident pos', 'Accident lane id']))
        return data
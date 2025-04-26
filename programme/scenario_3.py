#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：scenario_2.py 
@File    ：scenario_3.py
@Author  ：Lu
@Date    ：2023/5/29 9:56 
'''
'''
场景三：测试环境随机性
环境随机性的体现:
1) 车辆速度位置和类型随机：保证CAV的数量在 [N/2, N-2]之间, 位置随机但保证每个初始车辆之间的距离大于10
2) 瓶颈大小位置随机
'''
## Initialization module

import os
import sys
import plexe
import random
import numpy as np
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
from plexe import RADAR_DISTANCE, Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION
from functions import get_platoon_intensity, get_platoon_lane, clear_platoon_lane, \
    choose_leader, Join_platoon, create_accident, create_vehicle


def main(demo_mode, real_engine, setter=None):
    global platoon_lane, leader_id, accident_lane
    start_sumo("new cfg/highway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    topology = dict()
    while running(demo_mode, step, 10000):
        if demo_mode and step == 10000:
            print('仿真结束')
            traci.close()
            # start_sumo("new cfg/highway.sumo.cfg", True)
            step = 0

        traci.simulationStep()

        # create vehicles
        if step == 0:

            create_vehicle(5)

            traci.gui.trackVehicle("View #0", 'v.0.1')
            traci.gui.setZoom("View #0", 2000)
        if step == 200:
            accident_lane = create_accident(random.randint(1,5))


        if step % 10 == 1:
            communicate(plexe, topology)
        # 清理车排车道
        # if step % 200 == 0 and step > 600:
        #     clear_platoon_lane(platoon_lane)

        step += 1


    traci.close()


if __name__ == "__main__":
    main(True, False)
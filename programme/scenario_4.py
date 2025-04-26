#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：test.py 
@File    ：scenario_4.py
@Author  ：Lu
@Date    ：2023/6/20 10:07 
'''
'''
场景四描绘了窗口的选择

如何确定窗口大小以及窗口的车辆的ID？
1) 先选取距离瓶颈问题最近的CAV, 以该CAV的位置为一个边a
2) b = a - distance, 按照某个常量往前移动, 当窗口内的车辆达到一定数量限制时, 
   或者窗口达最大大小时或者b < 0时, 停止移动
返回窗口内的车辆ID列表, 以及窗口的两条边a, b
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
from ccparams import MAX_R_NUM, MAX_R_LENGTH

def get_acc_front_vid(accident_pos):
    # 获取瓶颈问题前最前方车辆的id和pos
    IDs = traci.vehicle.getIDList()
    front_vid = 'v.0.1'
    front_vid_pos = traci.vehicle.getLanePosition('v.0.1')
    for id in IDs:
        if id[2] == '0':
            vid_pos = traci.vehicle.getLanePosition(id)
            if vid_pos > front_vid_pos and vid_pos < accident_pos:
                front_vid = id
                front_vid_pos = vid_pos
    print('front_vid:',front_vid, 'front_vid_pos:', front_vid_pos)
    return front_vid

def get_vid_in_rectangle(accident_pos):
    # 获取瓶颈区域前最前方车辆的id和pos
    front_vid = get_acc_front_vid(accident_pos)
    front_pos = traci.vehicle.getLanePosition(front_vid)
    # 将f_pos作为第一个窗口的一条边a, 另一条边b动态往前移动
    a = front_pos
    b = front_pos - 50
    v_num = 0
    vid_in_rectangle = []
    # 窗口移动
    while(a - b <= MAX_R_LENGTH):
        print(b)
        IDs = traci.vehicle.getIDList()
        for id in IDs:
            if id[2] == '0':
                pos = traci.vehicle.getLanePosition(id)
                # print('vid:',id,'pos:',pos)
                if v_num < MAX_R_NUM:
                    if pos >= b and pos <= a:
                        # print('[',b,',',a,']')
                        if id not in vid_in_rectangle:
                            vid_in_rectangle.append(id)
                            v_num = v_num + 1
                            # print(v_num)
        if v_num >= MAX_R_NUM:
            break

        if b <= 0:
            break
        b = b - 50
        if b < 0:
            b = 0

    return vid_in_rectangle, a, b



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
            # traci.vehicle.add(vid, "platoon_route", departPos=str(position),
            #                   departSpeed=str(speed), departLane=str(lane),
            #                   typeID=vtype)
            # vid : v.0.0 for av , v.1.0 for hv
            # traci.vehicle.add(vehID='v.0.0', routeID="platoon_route", typeID="vtypeauto",
            #                   departPos=str(10), departLane=str(2), departSpeed=str(20.00))
            # traci.vehicle.add(vehID='v.0.1', routeID="platoon_route", typeID="vtypeauto",
            #                   departPos=str(25), departLane=str(1), departSpeed=str(20.00))
            # traci.vehicle.add(vehID='v.0.2', routeID="platoon_route", typeID="vtypeauto",
            #                   departPos=str(50), departLane=str(0), departSpeed=str(20.00))
            # traci.vehicle.add(vehID='v.1.0', routeID='platoon_route', typeID="passenger",
            #                   departPos=str(25), departLane=str(0), departSpeed=str(25.00))
            # traci.vehicle.add(vehID='v.1.1', routeID='platoon_route', typeID="passenger2",
            #                   departPos=str(70), departLane=str(2), departSpeed=str(25.00))
            create_vehicle(10)
            traci.gui.trackVehicle("View #0", 'v.0.1')
            traci.gui.setZoom("View #0", 2000)

        # 发生瓶颈问题时选择车排车道和排长
        if step == 200:
            # 产生瓶颈问题
            accident_lane, accident_pos = create_accident(vehicle_num = 5)
            print('accident_pos:', accident_pos)
        #
        if step == 500:
            R, a, b = get_vid_in_rectangle(accident_pos)
            print(R)
            print('a:',a,' b:',b)

        # if step == 300:
        #     print(traci.vehicle.getPosition('v.2.0'))
        #     print(traci.vehicle.getLanePosition('v.2.0'))

        if step % 10 == 1:
            communicate(plexe, topology)
        # 清理车排车道
        # if step % 200 == 0 and step > 600:
        #     clear_platoon_lane(platoon_lane)

        step += 1


    traci.close()


if __name__ == "__main__":
    main(True, False)
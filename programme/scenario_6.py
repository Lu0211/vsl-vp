#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@File    ：scenario_6.py
@Author  ：Lu
@Date    ：2023/6/24 15:50 
'''
'''
场景六测试SUMO中IDM模型的可行性

主要是测试IDM模型的距离感知
会不会很早的感知到前方出现了瓶颈问题, 正常情况下不应该过早感知到瓶颈问题再去变道
测试以下两个情况
1) 把一条车道堵上, 查看车辆到什么位置能感知到
2) 先把三条车道都堵上, 再解开, 查看后续车辆的行为
'''
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
    choose_leader, Join_platoon, create_accident, create_vehicle, get_vid_in_rectangle

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
            # vid : v.0.1 for av , v.1.1 for hv
            traci.vehicle.add(vehID='v.0.1', routeID="platoon_route", typeID="passenger",
                              departPos=str(10), departLane=str(2), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.0.2', routeID="platoon_route", typeID="passenger",
                              departPos=str(25), departLane=str(1), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.0.3', routeID="platoon_route", typeID="passenger",
                              departPos=str(50), departLane=str(0), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.1.1', routeID='platoon_route', typeID="passenger",
                              departPos=str(25), departLane=str(0), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.1.2', routeID='platoon_route', typeID="passenger",
                              departPos=str(70), departLane=str(2), departSpeed=str(20.00))
            # create_vehicle(5)
            traci.gui.trackVehicle("View #0", 'v.0.2')
            traci.gui.setZoom("View #0", 2000)

        # 发生瓶颈问题时选择车排车道和排长
        if step == 200:
            # 产生瓶颈问题
            parking_id = 'ParkAreaA'
            accident_pos = traci.parkingarea.getStartPos(parking_id)
            lane_id = traci.parkingarea.getLaneID(parking_id)
            lane = lane_id[3]
            pos = traci.parkingarea.getStartPos(parking_id)
            for i in range(3):
                vid = "v.2.%d" % i
                traci.vehicle.add(vehID=vid, routeID='platoon_route', departPos=str(pos + (i + 1) * 10),
                                  departLane=lane, departSpeed=str(0.00))
                traci.vehicle.setParkingAreaStop(vehID=vid, stopID=parking_id, duration=200)
            print('accident_lane:', lane, ' accident_pos:', accident_pos)

        step += 1


    traci.close()


if __name__ == "__main__":
    main(True, False)
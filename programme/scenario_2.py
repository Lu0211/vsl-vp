#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：DQN.py
@File    ：scenario_1.py
@IDE     ：PyCharm
@Author  ：Lu
@Date    ：2023/4/27 18:11
'''
'''
场景二描绘了五辆车分别行驶在三个车道上，两个HV三个CAV;
当发生瓶颈问题时，通过一定算法选择好车排车道和排长(其中的一个CAV);
再控制其他的CAV加入车排

如何选择车排车道？
1) 根据不同车道中HV的密度，HV的渗透率
2) 根据不同车道中车队的强度
选择车辆最少，且HV也最少的车道，都相同时优先右侧道路
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
    choose_leader, Join_platoon, create_accident


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
            traci.vehicle.add(vehID='v.0.0', routeID="platoon_route", typeID="vtypeauto",
                              departPos=str(10), departLane=str(2), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.0.1', routeID="platoon_route", typeID="vtypeauto",
                              departPos=str(25), departLane=str(1), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.0.2', routeID="platoon_route", typeID="vtypeauto",
                              departPos=str(50), departLane=str(0), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.1.0', routeID='platoon_route', typeID="passenger",
                              departPos=str(25), departLane=str(0), departSpeed=str(25.00))
            traci.vehicle.add(vehID='v.1.1', routeID='platoon_route', typeID="passenger2",
                              departPos=str(40), departLane=str(2), departSpeed=str(25.00))

            traci.gui.trackVehicle("View #0", 'v.0.0')
            traci.gui.setZoom("View #0", 2000)

        # 发生瓶颈问题时选择车排车道和排长
        if step == 200:
            # 产生瓶颈问题
            accident_lane, accident_pos = create_accident(vehicle_num=3)

        if step == 220:
            # 选择车排车道：根据车流量和密度以及该车道上是否有HV来选择车排车道
            platoon_lane = get_platoon_lane(accident_lane)

            # 选择排长
            leader_id = choose_leader(platoon_lane, plexe)
            # 设置速度模式
            traci.vehicle.setSpeedMode(leader_id, 0)
            # 记录拓扑结构
            topology[leader_id] = {}

        if step > 300:
            # topology = Join_platoon(plexe, join_id = 'v.0.1', leader_id = leader_id,
            #              front_id = leader_id, topology = topology,step = step, begin_join = 200)
            topology = Join_platoon(plexe, join_id = 'v.0.1', leader_id = leader_id,
                                    front_id = leader_id, topology = topology, step = step, begin_join = 400)
            topology = Join_platoon(plexe, join_id='v.0.0', leader_id=leader_id,
                                    front_id='v.0.1', topology=topology, step=step, begin_join = 400)

        if step % 10 == 1:
            communicate(plexe, topology)
            print(topology)
        # 清理车排车道
        if step % 200 == 0 and step > 600:
            clear_platoon_lane(platoon_lane)

        step += 1


    traci.close()


if __name__ == "__main__":
    main(True, False)
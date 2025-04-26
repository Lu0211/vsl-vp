#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：main.py 
@File    ：test.py
@Author  ：Lu
@Date    ：2023/4/16 16:28 
'''

## Initialization module

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
from plexe import RADAR_DISTANCE, Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION
from create_vehicles import create_platoon, create_hv, create_av
from join import Join_platoon
from ccparams import GOING_TO_POSITION, COLLISION
from state import State
from accident import Accident

def main(demo_mode, real_engine, setter=None):
    global COLLISION
    random.seed(1)
    # 开启sumo仿真
    start_sumo("new cfg/highway.sumo.cfg", False)
    # 以下设置可以使得每次 traci.simulationStep() 之后都调用一次 plexe
    plexe = Plexe()
    traci.addStepListener(plexe)
    # 仿真步数
    step = 0
    # 通信拓扑结构
    topology = dict()

    # 一些超参数
    JOIN_POSITION = 1
    N_VEHICLES = 4
    JOINER = "v.0.0"
    state = GOING_TO_POSITION

    while running(demo_mode, step, 6000):
        # 6000 step for 60 seconds
        if demo_mode and step == 6000:
            print('仿真结束')
            start_sumo("new cfg/highway.sumo.cfg", True)
            step = 0
            random.seed(1)
        # 一步步仿真
        traci.simulationStep()
        if step == 0:
            # create vehicles
            topology = create_platoon(plexe, pid=0, lane=0, n=4, real_engine=real_engine)
            create_av(plexe, vid=0, lane=1, pos=50, real_engine=real_engine)
            create_hv(plexe, vid=0, lane=2, pos=50, real_engine=real_engine)

            # create accident
            A = Accident(pos=500, lane=1, type=1)
            A.setAccident()

            # 仿真跟踪设置
            traci.gui.trackVehicle("View #0", 'p.0.0')
            traci.gui.setZoom("View #0", 5000)

        if step % 10 == 1:
            # 拓扑通信（排内）
            communicate(plexe, topology)
            # 获取状态
            # S = State(plexe, 'v.0.0')
            # S.get_state()

        # 开始执行动作 加入车排
        topology, state = Join_platoon(plexe, pid=0, jid=JOINER, Jpos=JOIN_POSITION, \
                                       step=step, state=state, topology=topology, N_v=N_VEHICLES)
        # 判断是否发生碰撞
        colliding_vehicles = traci.simulation.getCollidingVehiclesIDList()
        if len(colliding_vehicles) != 0:
            COLLISION = True
            print("Collisions detected:", colliding_vehicles)
        if COLLISION :
            print("发生碰撞!!!仿真结束")
            traci.close()


        step += 1
    traci.close()


if __name__ == "__main__":
    main(True, True)
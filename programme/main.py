#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：programme.py
@File    ：main.py
@IDE     ：PyCharm
@Author  ：Lu
@Date    ：2023/4/11 19:14
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
from ccparams import GOING_TO_POSITION, OPENING_GAP, JOIN_DISTANCE, COMPLETED, DISTANCE, SPEED
from state import State
from accident import Accident

# maneuver actors
JOIN_POSITION = 1
N_VEHICLES = 4
JOINER = "v.0.0"


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    # 具体着色是在 utils module 的 add_platooning_vehicle 函数中实现的
    random.seed(1)

    # 运行 SUMO 的配置文件，后边的参数 False / True 表示 SUMO server 是否已经在运行了。
    # 若为 False，则打开 SUMO 并加载配置文件
    # 若为 True，则重新加载配置文件
    # freeway.sumo.cfg 中仿真步长为 0.01s
    start_sumo("new cfg/highway.sumo.cfg", False)

    # 以下设置可以使得每次 traci.simulationStep() 之后都调用一次 plexe
    plexe = Plexe()
    traci.addStepListener(plexe)

    step = 0
    topology = dict()
    min_dist = 1e6
    state = GOING_TO_POSITION

    # 主循环
    while running(demo_mode, step, 6000):
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            print("Min dist: %f" % min_dist)
            start_sumo("new cfg/highway.sumo.cfg", True)
            step = 0
            random.seed(1)
        traci.simulationStep()
        # 仿真初始化阶段，构造含有 8 辆车的 platoon
        # 设置 GUI 中画面在整个仿真过程中始终聚焦在 v.0.0， 即头车
        # 镜头缩放参数 20000, 这个可以根据具体场景设置，使得镜头既不会拉的太近，也不会拉的太远。
        if step == 0:
            # create vehicles
            topology = create_platoon(plexe, pid=0, lane=0, n=N_VEHICLES, real_engine=real_engine)
            create_av(plexe, vid=0, lane=1, pos=50, real_engine=real_engine)

            A = Accident(pos=200, lane=1, type=1)
            A.setAccident()

            create_hv(plexe, vid=0, lane=2, pos=50, real_engine=real_engine)
            traci.gui.trackVehicle("View #0", 'p.0.0')
            traci.gui.setZoom("View #0", 5000)

        # 每隔 10 步车辆之间通信一次，获得其他车辆的位置、速度、加速度等信息
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            # 通信是指排内通信
            communicate(plexe, topology)
            S = State(plexe, 'v.0.0')
            # S.get_state()

        topology, state = Join_platoon(plexe, pid=0, jid=JOINER, Jpos=JOIN_POSITION, \
                                       step=step, state=state, topology=topology, N_v=N_VEHICLES)

        # 是否使用 plexe 中改进的更加逼真的引擎模型
        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        # # 记录在整个仿真过程中车辆间隔的最小距离，有需要的话可以随后进行分析
        # if step > 1:
        #     radar = plexe.get_radar_data("p.0.1")
        #     if radar[RADAR_DISTANCE] < min_dist:
        #         min_dist = radar[RADAR_DISTANCE]
        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, True)

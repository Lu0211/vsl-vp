#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@File    ：train.py
@IDE     ：PyCharm
@Author  ：Lu
@Date    ：2023/8/2 15:48
'''
import random

import numpy as np
import traci

'''
固定场景下的训练
场景描述: 三辆CAV, 两辆HV, 位置速度固定, 瓶颈问题的位置也固定
1) state: [['vid','pos','lane_id','speed'],...,[accident_lane, accident_pos, v_num, None]]
2) action: [1,0]
3) reward: 
'''
import plexe
import os
import sys
from functions import *
from ccparams import *
from datetime import datetime
from plexe import Plexe

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

def main(demo_mode):
    # 神经网络参数
    epsilon = 0
    is_rend = True
    ## 初步设置
    start_sumo("new cfg/highway.sumo.cfg", False,gui = is_rend)
    # 初始化设置
    plexe = Plexe()
    traci.addStepListener(plexe)
    # 输出文件名
    current_datetime = datetime.now()
    file_path = 'output/' + current_datetime.strftime('%y_%m_%d')
    # 仿真时间 6000 = 60s
    simulation_step = 6000
    # 训练回合数
    episode = 0
    episode_max = 50
    # 每次训练回合内的参数设定
    step = 0
    total_reward = 0
    total_loss = []
    join_state = dict()
    topology = dict()
    frist_action = True
    # 网络设置
    Q_network = get_model()
    Q_network.eval()
    Q_network.load('model/2099_model.h5', load_weights=True)
    ## 开始仿真
    while running(demo_mode, step, simulation_step):

        if demo_mode and step == simulation_step:
            episode = episode + 1
            print('episode: ', episode, '; total_reward: ', total_reward, '; total_loss: ', total_loss)
            with open(file_path + '_total_reward_test.txt', 'a') as file:
                file.write(f"Episode {episode}: Total Reward = {total_reward}, Total Loss = {total_loss}\n")
            print('------第', episode, '回合仿真结束------')
            total_loss = []
            if episode >= episode_max:
                print('-----训练结束-----')
                break
            # 每回合参数重置
            total_reward = 0
            step = 0
            join_state = dict()
            topology = dict()
            frist_action = True
            # 重新开启下个回合的训练
            start_sumo("new cfg/highway.sumo.cfg", True, gui=is_rend)

        traci.simulationStep() # 仿真一步步的开始

        # create vehicles
        if step == 0:
            # vid: v.0.1 for av, v.1.1 for hv
            traci.vehicle.add(vehID='v.0.1', routeID="platoon_route", typeID="vtypeauto",
                              departPos=str(10), departLane=str(2), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.0.2', routeID="platoon_route", typeID="vtypeauto",
                              departPos=str(25), departLane=str(1), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.0.3', routeID="platoon_route", typeID="vtypeauto",
                              departPos=str(50), departLane=str(0), departSpeed=str(20.00))
            traci.vehicle.add(vehID='v.1.1', routeID='platoon_route', typeID="passenger",
                              departPos=str(25), departLane=str(0), departSpeed=str(25.00))
            traci.vehicle.add(vehID='v.1.2', routeID='platoon_route', typeID="passenger2",
                              departPos=str(70), departLane=str(2), departSpeed=str(25.00))
            if is_rend:
                traci.gui.trackVehicle("View #0", 'v.0.2')
                traci.gui.setZoom("View #0", 2000)

        if step == 500:
            # 产生瓶颈问题
            accident_lane, accident_pos = create_accident(vehicle_num=5)
            # print('accident_lane:', accident_lane, '; accident_pos:', accident_pos)
        if step == 520:
            # 根据瓶颈问题选择leader
            platoon_lane = get_platoon_lane(accident_lane)
            leader_id = choose_leader(platoon_lane, plexe)
            topology[leader_id] = {}
            # 根据当前路况, 确定窗口内的车辆ID(CAV)
            veh_ids, a, b = get_vid_in_rectangle(accident_pos)
            # print('veh_ids: ', veh_ids)

        if step > 500 and step % 500 == 0:
            # 先判断是否是第一次执行动作, 如果不是第一次就需要存储元组
            if frist_action:
                frist_action = False
            else:
                # print('不是第一次执行动作')
                # 若不是第一次执行动作, 先获取next_state和reward在和上一次执行的动作以及对应的state存储到memory中
                next_state, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                             veh_ids=veh_ids, leader_id=leader_id)
                # reward
                reward, done = get_reward(veh_ids, accident_pos, join_state)
                total_reward += reward

            # 执行动作
            if len(join_state) == 0:
                print('join_state为空, 所有CAV动作不受限制')
                # 获取状态
                state, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                        veh_ids=veh_ids, leader_id=leader_id)
                # 输出动作
                action = get_action(state, Q_network, epsilon)
                action = [1, 1]
                print('state:', state, 'action:',action)
                # 执行动作
                join_state = perform_action(agent_ids, action, join_state, leader_id, plexe, topology)
                print('join_state:', join_state)
            else:
                print('join_state不为空, 表面以及有CAV执行过合并动作, 其他动作需要受限制')
                # 获取状态
                state, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                        veh_ids=veh_ids, leader_id=leader_id)
                # 输出动作
                action = get_action(state, Q_network, epsilon)
                # 检查动作是否需要限制, 如果之前已经执行过合并动作, 之后就要受限制; 或正在执行动作, 当合并距离足够时, 讲动作状态调整为已完成
                marked_action = marked_aciton(join_state, action, agent_ids, plexe)

                # 执行动作
                join_state = perform_action(agent_ids, marked_action, join_state, leader_id, plexe, topology)
                print('join_state:', join_state)
        # CAV之间的通信
        if step % 50 == 1:
            communicate(plexe, topology)

            # print('topology:', topology)
        # 清理车排车道
        # if step % 200 == 0 and step > 600:
        #     clear_platoon_lane(platoon_lane)
        step += 1

    traci.close()


if __name__ == "__main__":
    main(True)
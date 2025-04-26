#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：test.py 
@File    ：scenario_5.py
@Author  ：Lu
@Date    ：2023/6/20 10:40 
'''
import json

'''
场景五在尝试加入强化学习

先把场景固定在只有五辆车, 三辆CAV, 两辆HV, 位置速度固定, 瓶颈问题的位置也固定
1) state:  [['vid','pos','lane_id','speed'],...,[accident_lane, accident_pos, v_num, None]]
2) action: [1,0]
3) reward:
'''

import os
import sys
import plexe
import random
import numpy as np
# 调用 utils module，里面包含了 platooning 的中层实现函数
from utils import add_platooning_vehicle, start_sumo, running, communicate, add_hv_vehicle, get_distance
import tensorflow as tf
import tensorlayer as tl
from collections import deque
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
    choose_leader, Join_platoon, create_accident, create_vehicle, get_vid_in_rectangle, \
    get_state,get_platoon_last_vid
from functions import *
from ccparams import SPEED, GOING_TO_POSITION, OPENING_GAP, COMPLETED, DISTANCE, \
    COLLISION_REWARD, TIME_REWARD, EPSILON_MIN, EPSILON_DECAY

def main(demo_mode, real_engine, setter=None):
    global platoon_lane, leader_id, accident_lane, epsilon
    start_sumo("new cfg/highway.sumo.cfg", False,gui=True)
    plexe = Plexe()
    traci.addStepListener(plexe)
    # init
    simulation_step = 6000
    epsilon = 1.0
    memory = deque(maxlen=2000)
    print('memory初始化完毕')
    episode = 0
    episode_max = 100
    step = 0
    total_reward = 0
    join_state = dict()
    topology = dict()
    frist_action = True
    Q_network = get_model()
    Q_network.train()
    while running(demo_mode, step, simulation_step):

        if demo_mode and step == simulation_step:
            print('仿真结束')
            start_sumo("new cfg/highway.sumo.cfg", True, gui=True)
            step = 0
            print('episode:', episode)
            print('total_reward:', total_reward)
            with open('total_reward.txt','a') as file:
                file.write(f"Round {episode}: Total Reward = {total_reward}\n")
            total_reward = 0
            join_state = dict()
            topology = dict()
            frist_action = True
            epsilon = update_epsilon(epsilon)
            episode = episode + 1
            print('epsilon:', epsilon)
            if episode >= episode_max:
                print('训练结束')
                break

        traci.simulationStep()

        # create vehicles
        if step == 0:
            # traci.vehicle.add(vid, "platoon_route", departPos=str(position),
            #                   departSpeed=str(speed), departLane=str(lane),
            #                   typeID=vtype)
            # vid : v.0.1 for av , v.1.1 for hv
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
            # create_vehicle(5)
            traci.gui.trackVehicle("View #0", 'v.0.2')
            traci.gui.setZoom("View #0", 2000)

        # 发生瓶颈问题时选择车排车道和排长
        if step == 500:
            # 产生瓶颈问题
            accident_lane, accident_pos = create_accident(vehicle_num = 5)
            print('accident_lane:', accident_lane, ' accident_pos:', accident_pos)
        if step == 520:
            # 选择leader
            platoon_lane = get_platoon_lane(accident_lane)
            leader_id = choose_leader(platoon_lane, plexe)
            topology[leader_id] = {}
            veh_ids, a, b = get_vid_in_rectangle(accident_pos)
            print('veh_ids: ', veh_ids)
            # last_vid, last_pos = get_last_vid(veh_ids)
            # print('last_vid:',last_vid,'  last_pos:',last_pos)

        if step % 10 == 1:
            communicate(plexe, topology)
            # print(topology)

        if step > 500 and step % 500 == 0:
            if frist_action:
                frist_action = False
            else:
                print('不是第一个此执行动作')
                next_state, input_dim, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                        veh_ids=veh_ids, leader_id=leader_id)
                # 获取奖励reward
                ## CAV的平均速度
                avg_speed = get_avg_speed(veh_ids)
                ## 是否发生碰撞
                colliding_vehicles = traci.simulation.getCollidingVehiclesIDList()
                if len(colliding_vehicles) != 0:
                    print("发生碰撞, 碰撞车辆ID：",colliding_vehicles)
                    colliding_reward = COLLISION_REWARD
                else:
                    colliding_reward = 0
                ## 所有车辆是否通过瓶颈区域
                last_vid, last_pos = get_last_vid(veh_ids)
                if last_pos > accident_pos:
                    print('最后一辆车已通过瓶颈区域')
                    time_reward = 0
                    done = True
                else:
                    time_reward = TIME_REWARD
                    done = False

                reward = avg_speed + colliding_reward + time_reward
                # print('reward:',reward)
                total_reward += reward

                # print('state:',state,'action:',action,'-------next state:',next_state.flatten().tolist(),',reward:',reward)
                add_data_to_json('data.json', next_state.flatten().tolist())
                add_data_to_json('data.json', reward)
                add_data_to_json('data.json', done)

                memory = remember(memory, state, action, next_state, reward, done)
                # print('memory:',memory)
            # 每隔 2s 执行一次动作
            if len(join_state) == 0:
                print('join_state为空,所有CAV均可执行动作')
                state, input_dim, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                        veh_ids=veh_ids, leader_id=leader_id)
                # print('state:', state)

                # action out
                action = get_action(state, Q_network, epsilon)
                print('action:', action)

                # perform action
                join_state = perform_action(agent_ids, action, join_state, leader_id, plexe, topology)
            else:
                print('join_state不为空,动作需要限制')
                state, input_dim, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                        veh_ids=veh_ids, leader_id=leader_id)
                # print('agent_ids:',agent_ids)
                # print('state:', state)

                # action out
                action = get_action(state, Q_network, epsilon)
                print('action:', action)
                # print('join_state:', join_state)
                for key, item in join_state.items():
                    state_value = item['state']
                    if state_value == GOING_TO_POSITION:
                        join_state = Is_joined_platoon(join_state, plexe)
                        vid = int(key[4])
                        for i in range(len(agent_ids)):
                            if int(agent_ids[i]) == vid:
                                action[i] = 0
                    elif state_value == COMPLETED:
                        vid = int(key[4])
                        for i in range(len(agent_ids)):
                            if int(agent_ids[i]) == vid:
                                action[i] = 0
                print('marked action:',action)
                # perform action
                join_state = perform_action(agent_ids, action, join_state, leader_id, plexe, topology)
            # print('-------state:',state.flatten().tolist(),',action:',action)
            data_to_store = [state.flatten().tolist(), action]
            # 添加数据到文件末尾
            with open('data.json', 'r') as file:
                loaded_data = json.load(file)
            loaded_data.append(data_to_store)
            with open('data.json', 'w') as file:
                json.dump(loaded_data, file)



        # 检测是否完成合并, 未完成需要其他操作
        if step % 200 == 0 and step > 500:
            print('join_state:', join_state)
        #     join_state = Is_joined_platoon(join_state, plexe)


        # 清理车排车道
        # if step % 200 == 0 and step > 600:
        #     clear_platoon_lane(platoon_lane)

        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, False)
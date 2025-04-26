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

def get_reward(veh_ids, accident_pos, join_state):
    '''
    获取奖励, 分为三部分
    1) CAV的平均速度 speed
    2) 是否发生了碰撞
    3) 是否通过了瓶颈
    4) 是否完成了合并
    :param veh_ids: 窗口内CAV的id
    :return: reward, done
    '''
    # CAV的平均速度
    avg_speed = get_avg_speed(veh_ids)
    # 是否发生碰撞
    colliding_vehicles = traci.simulation.getCollidingVehiclesIDList()
    if len(colliding_vehicles) != 0:
        print("发生碰撞, 碰撞车辆ID：", colliding_vehicles)
        colliding_reward = COLLISION_REWARD
    else:
        colliding_reward = 0
    # 所有CAV是否通过瓶颈区域
    last_vid, last_pos = get_last_vid(veh_ids)
    if last_pos > accident_pos:
        print('最后一辆车已通过瓶颈区域')
        time_reward = 0
        done = True
    else:
        time_reward = TIME_REWARD
        done = False
    # 检测CAV是否执行了合并动作, 应尽早执行动作加入车队
    if not join_state:
        join_reward = JOIN_REWARD
    else:
        join_reward = 0

    reward = avg_speed + colliding_reward + time_reward + join_reward
    return  reward, done

def marked_aciton(join_state, action, agent_ids, plexe):
    '''
    检查CAV的动作状态, 看是否需要对动作施加限制, 内含两个功能
    1) 对正在执行且达到要求的动作, 将其动作状态调整为完成
    2) 对于已经执行且完成合并动作的CAV, 对其后的动作进行限制
    :param join_state:
    :param action:
    :return:
    '''
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
    return action

def process_data(memory, batch, Q_network, target_Q_network, gamma):
    '''
    加载数据, 用于训练
    :param memory:
    :return:
    '''
    data = random.sample(memory, batch)
    s = np.array([d[0] for d in data])
    a = [d[1] for d in data]
    s_ = np.array([d[2] for d in data])
    r = [d[3] for d in data]
    done = [d[4] for d in data]
    y = Q_network(np.array(s, dtype='float32'))
    y = y.numpy()
    Q1 = target_Q_network(np.array(s_, dtype='float32'))
    Q2 = Q_network(np.array(s_, dtype='float32'))
    next_action = np.argmax(Q2, axis=1)


    for i, (_, a, _, r, done) in enumerate(data):
        if done:
            target = r
        else:
            target = r + gamma * Q1[i][next_action[i]]
        target = np.array(target, dtype='float32')

        y[i][a] = target

    return s,y

def update_Q_network(s, y, Q_network, opt):
    with tf.GradientTape() as tape:
        Q = Q_network(np.array(s, dtype='float32'))
        loss = tl.cost.mean_squared_error(Q, y)
    grads = tape.gradient(loss, Q_network.trainable_weights)
    opt.apply_gradients(zip(grads,  Q_network.trainable_weights))

    return loss

def update_target_Q(Q_network, target_Q_network):
    '''
    Q网络学习完之后，需要把参数赋值到target_Q网络
    '''
    for i, target in zip(Q_network.trainable_weights, target_Q_network.trainable_weights):
        target.assign(i)

def train(Q_network, target_Q_network, gamma, memory, batch, opt, total_loss, episode):
    '''
    开始训练
    :param Q_network:
    :param target_Q_network:
    :param gamma:
    :param memory:
    :param batch:
    :param opt:
    :param total_loss:
    :param episode:
    :return:
    '''
    s, y = process_data(memory, batch, Q_network, target_Q_network, gamma)
    loss = update_Q_network(s, y, Q_network, opt)
    total_loss.append(loss)
    if (episode + 1) % 20 == 0:
        update_target_Q(Q_network, target_Q_network)



def main(demo_mode):
    # 神经网络参数
    epsilon = 1
    memory = deque(maxlen=2000)
    batch = 32
    gamma = 0.95
    learning_rate = 1e-3
    opt = tf.optimizers.Adam(learning_rate)
    is_rend = False
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
    episode_max = 2200
    # 每次训练回合内的参数设定
    step = 0
    total_reward = 0
    total_loss = []
    join_state = dict()
    topology = dict()
    frist_action = True
    # 网络设置
    Q_network = get_model()
    Q_network.train()
    target_Q_network = get_model()
    target_Q_network.eval()

    ## 开始仿真
    while running(demo_mode, step, simulation_step):

        if demo_mode and step == simulation_step:
            episode = episode + 1
            if episode == 10:
                Q_network.save(str(episode) + '_model.h5', True)
            if (episode + 1) % 700 == 0:
                Q_network.save(str(episode) + '_model.h5', True)
            # 记录数据
            print('episode: ', episode, '; total_reward: ', total_reward, '; total_loss: ', total_loss)
            with open(file_path + '_total_reward.txt', 'a') as file:
                file.write(f"Episode {episode}: Total Reward = {total_reward}, Total Loss = {total_loss}\n")
            print('------第', episode, '回合仿真结束------')
            total_loss = []
            if len(memory) > batch:
                train(Q_network, target_Q_network, gamma, memory, batch, opt, total_loss, episode)
            if episode >= episode_max:
                print('-----训练结束-----')
                break
            # 每回合参数重置
            total_reward = 0
            step = 0
            join_state = dict()
            topology = dict()
            frist_action = True
            # epsilon更新
            if (episode + 1) % 5 == 0:
                epsilon = update_epsilon(epsilon)
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

            # traci.gui.trackVehicle("View #0", 'v.0.2')
            # traci.gui.setZoom("View #0", 2000)

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
                # 存储元组
                memory = remember(memory, state, action, next_state, reward, done)
            # 执行动作
            if len(join_state) == 0:
                print('join_state为空, 所有CAV动作不受限制')
                # 获取状态
                state, agent_ids = get_state(accident_lane=accident_lane, accident_pos=accident_pos,
                                                        veh_ids=veh_ids, leader_id=leader_id)
                # 输出动作
                action = get_action(state, Q_network, epsilon)
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
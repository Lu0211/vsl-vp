#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：scenario_2.py 
@File    ：functions.py
@Author  ：Lu
@Date    ：2023/5/12 9:46 
'''
import json
import math
## Initialization module

import os
import sys
import plexe
import random
import numpy as np
import tensorflow as tf
import tensorlayer as tl
from collections import deque
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
from ccparams import *


def get_platoon_lane(accident_lane):
    """
    算法流程: 获取每个车道上的车辆数目以及HV的数目, 求出CAV所占该车道总车辆数的比例,
    选择比例最小的车道, 若比例相同, 优先最右侧车道
    其他思路: 可以设置为某一个范围内的车辆数, 而不是整个车道上的车辆数目
    :return: platoon_lane
    """
    # 选取车排车道
    # 获取每个车道的车辆数
    lane_id = traci.lane.getIDList()
    vehicleCount = [-1, -1, -1]
    vehicleCount_cav = [0, 0, 0]
    P_cav = [0, 0, 0]
    for i in range(3):
        # 获取每个车道上车辆数目
        # vehicleCount[i] = traci.lane.getLastStepVehicleNumber(lane_id[i])
        # 获取每个车道上HV车辆的数目
        vehicle_id = traci.lane.getLastStepVehicleIDs(lane_id[i])
        count = 0
        total = 0
        for id in vehicle_id:
            if id[2] == '0':
                count += 1
                total += 1
            elif id[2] == '1':
                total += 1
        vehicleCount[i] = total
        vehicleCount_cav[i] = count
        # 计算每个车道中HV所占的比例
        if i == int(accident_lane):
            P_cav[i] = -1000.0
        else:
            P_cav[i] = vehicleCount_cav[i] / vehicleCount[i]

    # print(vehicleCount)
    # print(vehicleCount_cav)
    # print("CAV市场渗透率:", P_cav)
    platoon_intensity = get_platoon_intensity()
    Max_P = max(P_cav)
    # 找到数组中最大值的索引（全部）
    max_indices = [i for i, x in enumerate(P_cav) if x == Max_P]
    if len(max_indices) == 1:
        platoon_lane = np.argmax(P_cav)
    elif len(max_indices) == 2:
        if platoon_intensity[max_indices[0]] >= platoon_intensity[max_indices[1]]:
            platoon_lane = max_indices[0]
        else:
            platoon_lane = max_indices[1]


    # 设置好车排车道
    platoon_lane_id = lane_id[platoon_lane]
    traci.lane.setDisallowed(laneID = platoon_lane_id, disallowedClasses = ['passenger'])

    print('车排车道已选好, ID为: ',end="")
    print(platoon_lane)
    return platoon_lane
    # 其他思路: 获取一定范围内的车道数

def get_platoon_intensity():
    """
    算法流程: 获取每个车道的车队强度: 车队(相互连接的)中的CAV占总CAV的比例
    :return: platoon_intensity
    """
    # 获取每个车道上的 id_list = [0, 1, 0]; 0 for HV, 1 for CAV, 从左到右
    lane_id = traci.lane.getIDList()
    platoon_intensity = [0, 0, 0]
    for i in range(3):
        id_list = []
        vehicle_id = traci.lane.getLastStepVehicleIDs(lane_id[i])
        # print(vehicle_id)
        total = 0 # 总共CAV的数量
        for j in vehicle_id:
            if j[2] == '0':
                id_list.append(1)
                total += 1
            elif j[2] == '1':
                id_list.append(0)
        # print(id_list) # 从左到右, 1 for CAV, 0 for HV
        length = len(id_list)
        count = 0 # 单个CAV的数量
        for j in range(length):
            if id_list[j] == 1 \
                    and (j == 0 or id_list[j - 1] == 0) \
                    and (j == length - 1 or id_list[j + 1] == 0):
                count += 1
        # print('---',count, total)
        if total == 0:
            platoon_intensity[i] = 0.0
        else:
            platoon_intensity[i] = (total - count) / total
    # print("车队强度：", platoon_intensity)
    return platoon_intensity

def clear_platoon_lane(platoon_lane):
    """
    用于清理车排车道, 检查上面是否有HV, 若有就控制它变到隔壁车道
    """
    # 若platoon_lane上有HV, 则需要控制它去其他车道
    lane_id = traci.lane.getIDList()
    platoon_lane_vid = traci.lane.getLastStepVehicleIDs(lane_id[platoon_lane])
    for id in platoon_lane_vid:
        if id[2] == '1':
            traci.vehicle.changeLane(vehID = id, laneIndex = (platoon_lane + 1) % 2, duration = 2.0)
            print('HV已移到其他车道')

def choose_leader(platoon_lane, plexe):
    """
        选择领导者, 车排车道最前方的车辆
        注: 需要考虑窗口, 一个窗口内一个排长, 以下代码还未考虑窗口
    """
    vehicle_id = traci.lane.getLastStepVehicleIDs(laneID = traci.lane.getIDList()[platoon_lane])
    frontVehID = None
    for vehID in vehicle_id:
        if vehID[2] == '0':
            if frontVehID is None or traci.vehicle.getLanePosition(vehID) > \
                    traci.vehicle.getLanePosition(frontVehID):
                frontVehID = vehID

    leader_id = frontVehID
    # 更改leader的颜色，并用plexe的接口对其模型进行限制
    traci.vehicle.setColor(typeID=leader_id, color=((160, 32, 240)))

    plexe.set_path_cacc_parameters(vid=leader_id, distance=5, xi=2, omega_n=1, c1=0.5)
    plexe.set_cc_desired_speed(leader_id, 33.33)
    plexe.set_acc_headway_time(leader_id, 1.5)

    plexe.use_controller_acceleration(leader_id, False)
    plexe.set_active_controller(vid=leader_id, controller=ACC)

    plexe.set_fixed_lane(vid = leader_id, lane = platoon_lane, safe = False)
    print('车队队长已选好, ID为:', leader_id)
    return leader_id

def Join_platoon(plexe, join_id, leader_id, front_id, step, topology, begin_join):
    """
    算法流程: 控制JOIN加入车排，现在只实现了加入一辆车的车排，因为车排中车辆一多还要考虑加入车排中的哪个位置
    :return: topology
    """
    # get in position 追赶前车
    # 注: 追赶前车这个函数最好只运行一次，避免和后面相冲突

    if step == begin_join:
        # 设定参数
        '''
        设置车辆速度模式。车辆的速度有 5 个影响因素
        https://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#speed_mode_.280xb3.29
        1\. safe speed
        2\. max accel
        3\. max speed
        4\. right of way at intersections
        5\. brake hard to avoid passing red light
        如果全都不考虑，则设置为 [0, 0, 0, 0, 0] = 0, 此时，车速完全由 traci 控制
        如果全都考虑，则设置为 [1, 1, 1, 1, 1] = 31
        如果只考虑 safe speed，则设置为 [0, 0, 0, 0, 1] = 1
        '''
        print('------------')
        traci.vehicle.setSpeedMode(join_id, 0)
        # plexe.set_path_cacc_parameters(join_id, 5, 2, 1, 0.5)
        # plexe.set_cc_desired_speed(join_id, SPEED)
        # plexe.set_acc_headway_time(join_id, 1.5)

        # get in position
        topology[join_id] = {"leader": leader_id, "front": front_id}
        plexe.set_cc_desired_speed(join_id, SPEED + 3)
        plexe.set_active_controller(join_id, FAKED_CACC)
        lane = traci.vehicle.getLaneIndex(leader_id)
        plexe.set_fixed_lane(join_id, lane=lane, safe=False)
        # print('change the lane and get to the position')



    # open the gap 若加入车排中间位置还需要打开空间
    # ...

    # completed

    if step > begin_join :
        if get_distance(plexe, join_id, front_id) < DISTANCE*2 + 5 and \
            get_distance(plexe, join_id, front_id) > DISTANCE * 2:
            # print('距离够了')
            # plexe.set_fixed_lane(join_id, lane = lane, safe = True)
            plexe.set_path_cacc_parameters(join_id, distance=5)
            plexe.set_active_controller(join_id, CACC)


    return topology

def create_accident(vehicle_num):
    """
    算法流程: 创建瓶颈问题, 传入产生的车辆数, 在所有的停车场中随机选择停车场
    注意停车场一定是提前导入并且存在的, 因为不能在仿真的过程中动态的生成停车场
    :return: accident_lane
    """
    IDs = traci.parkingarea.getIDList()
    parking_id = 'ParkAreaA'
    # parking_id = random.choice(IDs)
    accident_pos = traci.parkingarea.getStartPos(parking_id)
    lane_id = traci.parkingarea.getLaneID(parking_id)
    lane = lane_id[3]
    pos = traci.parkingarea.getStartPos(parking_id)
    for i in range(vehicle_num):
        vid = "v.2.%d" % i
        traci.vehicle.add(vehID = vid, routeID = 'platoon_route', departPos = str(pos + (i+1) * 10),
                          departLane = lane, departSpeed = str(0.00))
        traci.vehicle.setParkingAreaStop(vehID = vid, stopID = parking_id, duration = 200)
    return lane, accident_pos

def get_random_numbers(v_num):
    # 每隔7个数选择一个随机数, 而且保证每两个数之间的差值要大于7
    numbers = []
    lower_limit = 1
    for i in range(1, v_num+1):
        upper_limit = lower_limit + 6 + (i - 1) * 7
        # print('---', lower_limit, upper_limit)
        number = random.randint(lower_limit, upper_limit)
        numbers.append(number)
        lower_limit = number + 7  # 为了确保任何两个随机数之间的差大于10，我们在每次生成随机数后，将下限设置为当前随机数加上11。
    random.shuffle(numbers)
    return numbers

def random_split(number):
    # 将一个整数随机分成三部分，表示每个车道上的车辆数
    split1 = random.randint(0, number - 2)
    split2 = random.randint(0, number - split1 - 1)

    # 计算第三个部分
    split3 = number - split1 - split2

    return [split1, split2, split3]

def random_vtype(i, j):
    # 生成车辆类型，随机生成0和1的数组
    array = [0] * i + [1] * j
    random.shuffle(array)
    return array

def random_lane(i, j, k):
    array = [0] * i + [1] * j + [2] * k
    random.shuffle(array)
    return array

def create_vehicle(v_num):
    """
    算法流程: 创建车辆，随机类型、位置和速度
    类型：保证CAV的数量在 [N/2, N-2]之间,
    位置：随机但保证每个初始车辆之间的距离大于10
    :param v_num: 车辆总数
    """
    # 随机车辆总数的一半(向上取整)至车辆总数减二之间的CAV数量
    cav_num = random.randint(math.ceil(v_num/2),v_num - 2)
    hdv_num =  v_num - cav_num

    pos = get_random_numbers(v_num)
    print('random_pos:', pos)

    lane_vnum = random_split(v_num)
    print('lane_vnum:', lane_vnum)

    vtype = random_vtype(cav_num, hdv_num)
    print('vtype:', vtype)

    lane_id = random_lane(lane_vnum[0], lane_vnum[1], lane_vnum[2])
    print('lane_id', lane_id)


    cav_id = 1
    hdv_id = 1
    for i in range(v_num):
        vPos = pos[i]
        vLane = lane_id[i]
        vType = vtype[i]
        if vType == 0:
            vid = 'v.0.%d' % cav_id
            cav_id += 1
        else:
            vid = 'v.1.%d' % hdv_id
            hdv_id += 1
        print('vid:',vid,'pos:',vPos,'lane:',vLane,'vtype',vType)
        if vType == 0:
            traci.vehicle.add(vehID=vid, routeID="vehicles_route", typeID="vtypeauto",
                          departPos=str(vPos), departLane=str(vLane), departSpeed=str(random.randint(10,15)))
            # traci.vehicle.setLaneChangeMode(vehID=vid,lcm='SL2015')
        elif vType == 1:
            if random.random() >= 0.5:
                type = "passenger"
            else:
                type = "passenger2"
            traci.vehicle.add(vehID=vid, routeID="vehicles_route", typeID=type,
                              departPos=str(vPos), departLane=str(vLane), departSpeed=str(random.randint(10, 15)))

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
    # print('The_front_vid:',front_vid, 'the_front_vid_pos:', front_vid_pos)
    return front_vid

def get_vid_in_rectangle(accident_pos):
    """
    算法流程: 如何确定窗口大小以及窗口的车辆的ID？
            1) 先选取距离瓶颈问题最近的CAV, 以该CAV的位置为一个边a
            2) b = a - distance, 按照某个常量往前移动, 当窗口内的车辆达到一定数量限制时,
               或者窗口达最大大小时或者b < 0时, 停止移动
    :return: 窗口内车辆ID列表, 窗口的两条边界a, b
    """
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
        # print(b)
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

def get_state(accident_lane, accident_pos, veh_ids, leader_id):
    """
    函数功能: 获取当前时刻t的状态state
    :param accident_pos: 事故发生位置
    :param veh_ids: 所要获取CAV车辆状态的列表
    :param leader_id: 领导者vid, 放在二维状态的第一行
    state = [['vid','pos','lane_id','speed'],...,[accident_lane, accident_pos, v_num, None]]
    :return: 二维向量 state
    """
    platoon_lane = traci.vehicle.getLaneIndex(leader_id)
    # print('platoon_lane:', platoon_lane)
    # print('leader_id:', leader_id)
    if leader_id in veh_ids:
        veh_ids.remove(leader_id)

    # print('veh_ids: ', veh_ids)
    v_num = len(veh_ids)
    agent_ids = []
    state = [[int(leader_id[4]), traci.vehicle.getLanePosition(leader_id),
              traci.vehicle.getLaneIndex(leader_id),
              traci.vehicle.getSpeed(leader_id)]]
    for id in veh_ids:
        agent_ids.append(int(id[4]))
        pos = traci.vehicle.getLanePosition(id)
        lane_index = traci.vehicle.getLaneIndex(id)
        speed = traci.vehicle.getSpeed(id)
        s = [int(id[4]), pos, lane_index, speed]
        state.append(s)
    state.append([accident_lane, accident_pos])

    flattened_state =  [item for sublist in state for item in sublist]
    # print('state:', state)
    # s = np.array(state)
    # print('s:', s)
    # h,l = s.shape
    # input_dim = h * l
    # s = s.reshape(-1, input_dim)
    # print('s:', s)
    # print('input_dim',input_dim)

    return flattened_state, agent_ids

def get_platoon_order(graph):
    '''
    获取车队中的次序
    :param graph: 车队拓扑结构
    :return: 车队中的次序
    '''
    in_degrees = {node: 0 for node in graph}
    for node in graph:
        for neighbor in graph[node].values():
            in_degrees[neighbor] += 1

    queue = deque([node for node in graph if in_degrees[node] == 0])
    result = []

    while queue:
        node = queue.popleft()
        result.append(node)
        for neighbor in graph[node].values():
            in_degrees[neighbor] -= 1
            if in_degrees[neighbor] == 0:
                queue.append(neighbor)
    return result

def get_platoon_last_vid(topology):
    '''
    根据当前拓扑结构,获取最后一辆车的id
    :param topology: 拓扑结构
    :return: last id
    '''
    order = get_platoon_order(topology)
    return order[0]

def get_model():
    '''
    获取模型, 构建神经网络基本结构
    输入state, 输出action
    :return: Model
    '''
    # DQN
    input = tl.layers.Input(shape = (None, 14))
    h1 = tl.layers.Dense(64, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(input)
    h2 = tl.layers.Dense(32, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(h1)
    output = tl.layers.Dense(4, act=tf.nn.sigmoid, W_init=tf.initializers.GlorotUniform())(h2)
    return tl.models.Model(inputs=input, outputs=output)

def decimal_to_binary_list(decimal_num, num_bits):
    '''
    将十进制代表的动作转化为二进制的数组,
    例如 0 -> [0, 0]; 1 -> [0, 1]
    :param decimal_num: 十进制
    :param num_bits: 转化的位数, 即列表长度或智能体数量
    :return: 列表, action
    '''
    binary_str = format(decimal_num, f'0{num_bits}b')
    binary_list = [int(bit) for bit in binary_str]
    return binary_list

def get_action(state, Q_network, epsilon):
    '''
    获取动作, 按照epsilon探索策略
    :param state:
    :param Q_network:
    :return: action(list)
    '''
    if np.random.rand() >= epsilon:
        print('---非随机---')
        q = Q_network(np.array(state, dtype='float32').reshape([-1, 14]))
        a = np.argmax(q)
    # 否则，随机一个动作输出。
    else:
        a = np.random.randint(0, 3)
    action = decimal_to_binary_list(a, num_bits = 2)
    return action

def update_action_list(action_list):
    '''
    动作列表的更新和规范化, 注意动作列表是无序的,
    需要控制车辆加入车队的顺序, 让离leader近的车辆先行加入
    根据智能体的位置将action_list排序, 智能体pos越大, 越先执行该动作
    :param action_list:
    :return: action_list
    '''
    sorted_list = sorted(action_list, key=lambda x:x[-1], reverse=True)
    return sorted_list

def get_in_position(plexe, join_id, front_id, leader_id, topology):
    '''
    控制vid逼近车队, 开始加入
    :param plexe:
    :param join_id: 加入车辆的id
    :param front_id: 加入车辆前方车辆的id
    :param leader_id: 领导者id
    :param topology:
    :return:
    '''
    traci.vehicle.setSpeedMode(join_id, 0)
    # 更改拓扑结构
    topology[join_id] = {"leader": leader_id, "front": front_id}
    plexe.set_cc_desired_speed(join_id, SPEED + 3)
    plexe.set_active_controller(join_id, FAKED_CACC)
    lane = traci.vehicle.getLaneIndex(leader_id)
    plexe.set_fixed_lane(join_id, lane=lane, safe=False)
    print('change the lane and get to the position')
    return topology, GOING_TO_POSITION

def perform_action(agent_ids, action, join_state, leader_id, plexe, topology):
    '''
    根据智能体以及他们的相应动作去执行
    :param agent_ids: 智能体列表
    :param action: 动作列表, 与智能体列表一一对应
    :param join_state: 智能体执行动作的状态, 因为动作并不是瞬间完成的, 后续要需要检测是否完成
    :param plexe: plexe工具包
    :param topology: 车队的拓扑结构
    :return:
    '''
    # 先将智能体执行动作的次序排好, 靠前的智能体先执行动作
    action_list = []
    for i in range(len(agent_ids)):
        vid = 'v.0.%d' % int(agent_ids[i])
        agent_pos = traci.vehicle.getLanePosition(vid)
        a = action[i]
        action_list.append([vid, a, agent_pos])
    action_list = update_action_list(action_list)
    print('action_list:', action_list)

    # 遍历动作列表, 执行动作, 开始追击加入车队, 将追击的车辆记录下来, 后续再检测是否完成加入
    for a in action_list:
        if a[1] == 1:
            vid = a[0]
            fid = get_platoon_last_vid(topology)
            topology, v_state = get_in_position(plexe, vid, fid, leader_id, topology)
            join_state[vid] = {'state': v_state, 'front_id': fid}
            print(vid, '正在追', fid, ',leader为', leader_id)

    action_list.clear()
    return join_state

def add_data_to_json(file, data):
    '''
    将数据添加到文件的最后一行
    :param file:
    :param data:
    :return:
    '''
    # 从文件中读取JSON数据
    with open('data.json', 'r') as file:
        loaded_data = json.load(file)

    last_list = loaded_data[-1]
    last_list.append(data)

    with open('data.json', 'w') as file:
        json.dump(loaded_data, file)

def update_epsilon(epsilon):
    '''
    用于更新epsilon, 除非epsilon已经足够小
    :param epsilon:
    :return:
    '''
    if epsilon >= EPSILON_MIN:
        epsilon *= EPSILON_DECAY
    return epsilon

def Is_joined_platoon(join_state, plexe):
    '''
    检测执行动作的智能体CAV是否完成了合并操作,
    当距离合适时, 提示完成合并
    :param join_state: 动作执行的状态(动作并不是瞬间完成, 存在状态的转移)
    :return: join_state
    '''
    if join_state:
        for i in join_state.keys():
            v_state = join_state[i]['state']
            fid = join_state[i]['front_id']
            if v_state == GOING_TO_POSITION:
                if get_distance(plexe, i, fid) < DISTANCE * 2:
                    print('距离够了, 完成合并操作')
                    plexe.set_path_cacc_parameters(i, distance=5)
                    plexe.set_active_controller(i, CACC)
                    join_state[i]['state'] = COMPLETED
                    break

    return join_state

def get_last_vid(vids):
    """
    获取车辆id列表中, 位置最靠后的vid
    :param vids: 车辆列表ids
    :return: last_vid
    """
    last_pos = None
    last_vid = None
    for id in vids:
        pos = traci.vehicle.getLanePosition(id)
        if last_pos is None or pos < last_pos:
            last_vid = id
            last_pos = pos
    return last_vid, last_pos

def get_avg_speed(vids):
    '''
    获取所有CAV的平均速度
    :param vids:
    :return:
    '''
    total_speed = 0
    for id in vids:
        total_speed += traci.vehicle.getSpeed(id)
    avg_speed = total_speed / len(vids)
    return avg_speed

def remember(memory, s, a, s_, r, done):
    '''
    将数据存储到experience replay中
    :param s:
    :param a:
    :param s_:
    :param r:
    :param done:
    :return: memory
    '''
    data = (s, a, s_, r, done)
    memory.append(data)
    memory_str = '\n'.join(str(item) for item in memory)
    out_file = 'output/output.txt'
    with open(out_file, 'w') as file:
        file.write(memory_str)
    return memory

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
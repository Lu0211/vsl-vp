#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：Highway.py
@File    ：Highway.py
@Author  ：Lu
@Date    ：2023/8/30 10:32 
'''
import random
from collections import deque
import traci
import tensorflow as tf
import tensorlayer as tl
import sys
sys.path.append('H:\Lu_programme\programme')
import numpy as np
import plexe
from plexe import Plexe
from plexe import ACC, CACC, FAKED_CACC
from datetime import  datetime
from ccparams import *
from utils import *

class Highway():
    def __init__(self):
        self.input_dim = 27 # 5*5 + 2

        # 建立两个网络
        self.Q_network = self.get_model()  # 建立一个Q网络
        self.Q_network.train()  # 在tensorlayer要指定这个网络用于训练。
        self.target_Q_network = self.get_model()  # 创建一个target_Q网络
        self.target_Q_network.eval()  # 这个网络指定为不用于更新。

        # epsilon-greedy相关参数
        self.epsilon = 1.0  # epsilon大小，随机数大于epsilon，则进行开发；否则，进行探索。
        self.epsilon_decay = 0.98  # 减少率：epsilon会随着迭代而更新，每次会乘以0.995
        self.epsilon_min = 0.05  # 小于最小epsilon就不再减少了。

        # 其余超参数
        self.memory = deque(maxlen=200000)  # 队列，最大值是200000
        self.batch = 512
        self.gamma = 0.95  # 折扣率
        self.learning_rate = 1e-3  # 学习率
        self.opt = tf.optimizers.Adam(self.learning_rate)  # 优化器
        self.is_rend = True # 是否可视化仿真

        # plexe相关参数
        start_sumo("cfg/highway.sumo.cfg", False, gui = self.is_rend)
        self.plexe = Plexe()
        traci.addStepListener(self.plexe)

        # 仿真相关参数
        ## 输出文件名
        current_datetime = datetime.now()
        self.file_path = 'output/' + current_datetime.strftime('%y_%m_%d')
        ## 仿真时间 10000 == 100s
        self.simulation_step = 15000
        ## 训练回合数
        self.episode_max = 1
        self.episode = 0
        ## 每次训练回合内的参数设定
        self.step = 0
        self.total_reward = 0
        self.total_loss = []
        self.leader_ids = []
        self.collision_vids = []
        self.join_state = dict()
        self.change_lane_state = dict()
        self.topology = dict()
        self.frist_action = True
        self.collision = False
        self.done = False

        # 路段参数
        self.lane_ids = ['E0', 'E1', 'E2']
        self.total_vids = []
        self.start_vids = []    # 记录开始合并的cavs
        self.window_vids = []    # 窗口记录每个截取窗口内的vids
        self.window_cav_vids = []       # 窗口记录每个截取窗口内的cav vids
        self.platoon_lane = -1
        self.accident_lane = 1
        self.accident_pos = 1000
        self.vsl_lane = ['E1']
        # 开始合并的路段
        self.start_platoon_lane = 'E2'
        # 刚驶入合并路段的一组车辆, 每五辆一组
        self.platooning_vids = []

    def reinit(self):
        '''
        重置参数, 继续下一次仿真
        '''
        self.step = 0
        self.total_reward = 0
        self.total_loss = []
        self.leader_ids = []
        self.collision_vids = []
        self.join_state = dict()
        self.change_lane_state = dict()
        self.topology = dict()
        self.frist_action = True
        self.collision = False
        self.done = False

        self.total_vids = []
        self.start_vids = []  # 记录开始合并的cavs
        self.window_vids = []  # 窗口记录每个截取窗口内的vids
        self.window_cav_vids = []  # 窗口记录每个截取窗口内的cav vids
        self.platoon_lane = -1
        self.accident_lane = 1

    def update_epsilon(self):
        '''
        更新epsilon, 除非epsilon已经足够小
        '''
        if self.epsilon >= self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def get_model(self):
        '''
            获取模型, 构建神经网络基本结构
            输入state, 输出action
            '''
        # DQN
        self.input = tl.layers.Input(shape=(None, self.input_dim))
        self.h1 = tl.layers.Dense(64, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(self.input)
        self.h2 = tl.layers.Dense(32, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(self.h1)
        self.output = tl.layers.Dense(4, act=tf.nn.sigmoid, W_init=tf.initializers.GlorotUniform())(self.h2)
        return tl.models.Model(inputs=self.input, outputs=self.output)

    def get_model_N(self, action_size):
        """
        根据输出动作的不同, 动态调整输出层
        """
        # DQN
        self.input = tl.layers.Input(shape=(None, self.input_dim))
        self.h1 = tl.layers.Dense(16, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(self.input)
        self.h2 = tl.layers.Dense(16, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(self.h1)
        self.output = tl.layers.Dense(2 ** action_size, act=tf.nn.sigmoid, W_init=tf.initializers.GlorotUniform())(self.h2)
        return tl.models.Model(inputs=self.input, outputs=self.output)

    def get_platoon_intensity(self):
        '''
        算法流程: 获取每个车道的车队强度: 车队(相互连接的)中的CAV占总CAV的比例
        lane_id: 路段ID -> E0/E1 ...
        :return: platoon_intensity
        '''
        # 获取每个车道上的 id_list = [0, 1, 0]; 0 for CAV, 1 for HDV, 从左到右
        platoon_intensity = [0, 0, 0]
        lane_ids = self.lane_ids
        for i in range(3):
            id_list = []
            vehicle_id = []
            for x in range(3):
                vehicle_id.append(traci.lane.getLastStepVehicleIDs(lane_ids[x] + '_' + str(i)))
            vehicle_id =  [item for sublist in vehicle_id for item in sublist]
            # print(vehicle_id)
            total = 0  # 总共CAV的数量
            for j in vehicle_id:
                if j[2] == '0':
                    id_list.append(1)
                    total += 1
                elif j[2] == '1':
                    id_list.append(0)
            # print(id_list) # 从左到右, 1 for CAV, 0 for HV
            length = len(id_list)
            count = 0  # 单个CAV的数量
            for j in range(length):
                if id_list[j] == 1 \
                        and (j == 0 or id_list[j - 1] == 0) \
                        and  (j == length - 1 or id_list[j + 1] == 0):
                    count += 1
            # print('---',count, total)
            if total == 0:
                platoon_intensity[i] = 0.0
            else:
                platoon_intensity[i] = (total - count) / total
        print("车队强度：", platoon_intensity)
        return platoon_intensity

    def get_platoon_lane(self):
        '''
        算法流程: 获取每个车道上的车辆数目以及HV的数目, 求出CAV所占该车道总车辆数的比例,
        选择比例最小的车道, 若比例相同, 优先最右侧车道
        其他思路: 可以设置为某一个范围内的车辆数, 而不是整个车道上的车辆数目
        '''
        # 选取车排车道
        # 获取每个车道的车辆数
        if self.platoon_lane >= 0:
            return
        lane_ids = self.lane_ids
        vehicleCount = [-1, -1, -1]
        vehicleCount_cav = [0, 0, 0]
        P_cav = [0, 0, 0]
        for i in range(3):
            # 获取每个车道上车辆数目
            # vehicleCount[i] = traci.lane.getLastStepVehicleNumber(lane_id[i])
            # 获取每个车道上HV车辆的数目
            vehicle_id = []
            for x in range(3):
                vehicle_id.append(traci.lane.getLastStepVehicleIDs(lane_ids[x] + '_' + str(i)))
            vehicle_id = [item for sublist in vehicle_id for item in sublist]
            # print(vehicle_id)
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
            if i == int(self.accident_lane):
                P_cav[i] = -1000.0
            elif vehicleCount[i] == 0:
                P_cav[i] = 1000.0
            else:
                P_cav[i] = vehicleCount_cav[i] / vehicleCount[i]

        # print(vehicleCount)
        # print(vehicleCount_cav)
        print("CAV市场渗透率:", P_cav)
        platoon_intensity = self.get_platoon_intensity()
        Max_P = max(P_cav)
        # print(Max_P)
        # 找到数组中最大值的索引（全部）
        max_indices = [i for i, x in enumerate(P_cav) if x == Max_P]
        if len(max_indices) == 1:
            platoon_lane = np.argmax(P_cav)
        elif len(max_indices) == 2:
            if platoon_intensity[max_indices[0]] >= platoon_intensity[max_indices[1]]:
                platoon_lane = max_indices[0]
            else:
                platoon_lane = max_indices[1]
        # # 设置好车排车道

        platoon_lane_id = self.start_platoon_lane + '_' + str(platoon_lane)
        traci.lane.setDisallowed(laneID=platoon_lane_id, disallowedClasses=['passenger'])

        print('车排车道已选好, ID为: ', end="")
        print(platoon_lane)
        self.platoon_lane = platoon_lane
        # 其他思路: 获取一定范围内的车道数

    def clear_platoon_lane(self):
        '''
        清理车队的专用车道, 尽量不让HDV在此车道上行驶
        '''
        lane_ids = ['E1', 'E2']
        for id in lane_ids:
            lane_id = id + '_' + str(self.platoon_lane)
            platoon_lane_vid = traci.lane.getLastStepVehicleIDs(laneID = lane_id)
            for vid in platoon_lane_vid:
                if vid[2] == '1':
                    traci.vehicle.changeLane(vehID = vid, laneIndex = (self.platoon_lane + 1) % 2, duration = 2.0)

    def create_accident(self):
        '''
        创建瓶颈问题, 即一些发生事故的拥堵车辆
        '''
        parking_id = 'ParkAreaA'
        accident_pos = traci.parkingarea.getStartPos(parking_id)
        lane_id = traci.parkingarea.getLaneID(parking_id)
        lane_index = lane_id[3]
        for i in range(5):
            vid = 'v.2.%d' % i
            traci.vehicle.add(vehID=vid, routeID='accident_route', departPos=str(accident_pos + (i + 1) * 10),
                              departLane=lane_index, departSpeed=str(0.00))
            traci.vehicle.setParkingAreaStop(vehID=vid, stopID=parking_id, duration=200)
        self.accident_lane = lane_index
        self.accident_pos = accident_pos + 1000

    def create_vehicles(self):
        """
        添加车辆
        """
        max_cav_value, max_hdv_value = self.get_max_vid_value()
        if max_cav_value == -1:
            max_cav_value = 0
        if max_hdv_value == -1:
            max_hdv_value = 0
        # print('max_cav_value:', max_cav_value, 'max_hdv_value:', max_hdv_value)

        cav_pos = [10, 3, 30, 53, 64, 60, 88, 105, 130]
        cav_index = [1, 2, 0, 2, 0, 1, 1, 2, 0]
        hdv_pos = [4, 33, 40, 85, 100, 125]
        hdv_index = [0, 2, 1, 2, 0, 1]

        for i in range(len(cav_pos)):
            cav_vid = 'v.0.' + str(max_cav_value + i + 1)
            pos = cav_pos[i]
            index = cav_index[i]
            traci.vehicle.add(vehID=cav_vid, routeID="highway_route", typeID="vtypeauto",
                              departLane= index, departPos=pos, departSpeed=20.00)

        for j in range(len(hdv_pos)):
            hdv_vid = 'v.1.' + str(max_hdv_value + j + 1)
            pos = hdv_pos[j]
            index = hdv_index[j]
            traci.vehicle.add(vehID=hdv_vid, routeID="highway_route", typeID="passenger",
                              departLane=index, departPos=pos, departSpeed=20.00)

    def generate_random_matrix(self, rows, cols):
        '''
        生成随机的CAV类型
        大小为 rows * cols
        要求每一行中必须都有 0->CAV
        且CAV总数要占百分之60
        '''
        matrix = []

        for i in range(rows):
            row = [1] * cols  # 先将整行初始化为1（HDV）
            random_index = random.randint(0, cols - 1)
            row[random_index] = 0  # 随机选择一个位置设置为0（CAV）
            matrix.append(row)

        # 随机选择一些位置设置为0（CAV），以满足总数的60%要求
        num_zeros = int(rows * cols * 0.6) - rows
        zero_positions = [(i, j) for i in range(rows) for j in range(cols) if matrix[i][j] == 1]
        random.shuffle(zero_positions)

        prev_zero_row = -1
        prev_zero_col = -1

        for i in range(min(num_zeros, len(zero_positions))):
            row, col = zero_positions[i]

            # 检查前一个位置是否已经是0，如果是的话，选择下一个位置
            if row == prev_zero_row and col == prev_zero_col:
                i += 1
                row, col = zero_positions[i]

            matrix[row][col] = 0
            prev_zero_row = row
            prev_zero_col = col

        return matrix

    def create_vehicles_random(self):
        """
        带有随机性创建车辆
        num = 15
        """
        max_cav_value, max_hdv_value = self.get_max_vid_value()
        if max_cav_value == -1:
            max_cav_value = 0
        if max_hdv_value == -1:
            max_hdv_value = 0
        # print(max_cav_value, max_hdv_value)
        # position = [[3, 4, 10], [30, 33, 40], [53, 60, 64], [85, 88, 100], [105, 100, 125]]
        # lane_index = [[2, 1, 0], [0, 2, 1], [2, 1, 0], [2, 1, 0], [2, 0, 1]]
        position = [[3, 4, 8], [33, 34, 40], [63, 66, 74], [95, 107, 100], [140, 130, 135]]
        lane_index = [[2, 1, 0], [0, 2, 1], [2, 1, 0], [2, 1, 0], [2, 0, 1]]
        # vtype = [[1, 0, 0], [0, 1, 1], [0, 0, 1], [1, 1, 0], [0, 0, 0]]
        # vtype = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 1], [1, 0, 1]]
        vtype = [[0, 0, 0], [0, 1, 0], [1, 1, 0], [0, 1, 1], [1, 0, 0]]
        # vtype = self.generate_random_matrix(5, 3)
        #
        print(vtype)
        for i in range(len(position)):
            for j in range(len(position[i])):
                pos = position[i][j]
                index = lane_index[i][j]
                type = vtype[i][j]
                if type == 0:
                    vid = 'v.0.' + str(1 + max_cav_value)
                    max_cav_value += 1
                    traci.vehicle.add(vehID=vid, routeID="highway_route", typeID="vtypeauto",
                                      departLane=index, departPos=pos, departSpeed=20.00)
                elif type == 1:
                    vid = 'v.1.' + str(1 + max_hdv_value)
                    max_hdv_value += 1
                    traci.vehicle.add(vehID=vid, routeID="highway_route", typeID="passenger",
                                      departLane=index, departPos=pos, departSpeed=20.00)

    def vsl(self):
        '''
        开启VSL区域限制
        '''
        vids = self.get_total_vids()
        for id in vids:
            lane_id = traci.vehicle.getLaneID(id)
            lane = lane_id[:2]
            if lane in self.vsl_lane:
                traci.vehicle.setMaxSpeed(id, speed = 25.00)
            else:
                if id[2] == '0':
                    if id not in self.leader_ids:
                        traci.vehicle.setMaxSpeed(id, speed = DESIRED_SPEED)
                elif id[2] == '1':
                    traci.vehicle.setMaxSpeed(id, speed = DESIRED_SPEED)

    def get_max_vid_value(self):
        """
        获取当前存在车辆的最大id, 为了之后创建车辆
        """
        current_vids = traci.vehicle.getIDList()

        max_cav_value = -1
        max_hdv_value = -1
        max_cav_id = None
        max_hdv_id = None

        for vid in current_vids:
            if vid.startswith('v.0.'):
                cav_value = int(vid.split('.')[-1])
                if cav_value > max_cav_value:
                    max_cav_value = cav_value
                    max_cav_id = vid
            elif vid.startswith('v.1.'):
                hdv_value = int(vid.split('.')[-1])
                if hdv_value > max_hdv_value:
                    max_hdv_value = hdv_value
                    max_hdv_id = vid

        return max_cav_value, max_hdv_value

    def get_vid_posX(self, vid):
        '''
        获取每个车辆的x位置
        '''
        pos = traci.vehicle.getPosition(vid)
        return pos[0]

    def get_total_vids(self):
        '''
        获取仿真中所有车辆的ID, 按位置进行排序
        '''
        vids = traci.vehicle.getIDList()
        vids = [item for item in vids if not item.startswith('v.2.')]
        vids = sorted(vids, key=self.get_vid_posX)
        self.total_vids = vids

        return vids

    def get_lane_vids(self, lane_id):
        '''
        获取该路段上车辆ID, 按位置进行排序
        '''
        vids = []
        for lane_index in range(3):
            id = lane_id + '_' + str(lane_index)
            vids.append(traci.lane.getLastStepVehicleIDs(id))
        vids = [item for sublist in vids for item in sublist]
        # print(vids)
        vids = sorted(vids, key=self.get_vid_posX)
        # print(vids)
        vids = [x for x in vids if x[2] != '2']
        return vids

    def sliding_windows(self, N, R):
        """
        滑动窗口算法
        N -> max num of vehicles within a window
        R -> max CAV communication distance
        1) 动态窗口: 每次执行编队算法的时候都执行窗口分组
        2) 静态窗口: 分完一次组后不再更新
        """
        windows = []
        current_window = []
        vids = self.get_total_vids()
        for vid in vids[::-1]:
            vtype = vid[2]
            pos = traci.vehicle.getPosition(vid)[0]
            if not current_window:
                if vtype == '0':
                    current_window.append(vid)
                else:
                    current_window.append(vid)
            elif len(current_window) < N:
                if vtype == '0':
                    last_cav_id = '00'
                    for id in current_window:
                        if id[2] == '0':
                            last_cav_id = id
                    if last_cav_id != '00':
                        last_cav_pos = traci.vehicle.getPosition(last_cav_id)[0]
                        if abs(last_cav_pos - pos) <= R:
                            current_window.append(vid)
                        else:
                            windows.append(current_window)
                            current_window = []
                            current_window.append(vid)
                    else:
                        current_window.append(vid)
                else:
                    current_window.append(vid)

            if len(current_window) == N:
                windows.append(current_window)
                current_window = []

        if current_window:
            windows.append(current_window)

        return windows

    def get_vids_in_rectangle(self, N, R):
        """
        刚驶入E2路段的车辆, 每五辆一组开始合并
        注: 为加上距离限制, 若前五辆车之间位置相差很多, 则make no sense
        """
        vids = self.get_lane_vids(lane_id = 'E2')
        if len(vids) < 5:
            print('车辆不够')
            return 0
        # for i in range(3):
        #     land_id = 'E' + str(i)
        vids = self.get_total_vids()
        vids = [x for x in vids if all(x not in row for row in self.window_vids)]
        positions = []
        for vid in vids[::-1]:
            positions.append(traci.vehicle.getPosition(vid)[0])

        current_group = []

        for vid, pos in zip(vids[::-1], positions):
            vtype = vid[2]  # type 0 for CAV, 1 for HDV
            if not current_group:
                if vtype == '0':
                    current_group.append(vid)
                else:
                    continue
            elif len(current_group) < N:
                if vtype == '0':
                    for id in current_group:
                        if id[2] == '0':
                            last_cav_id = id
                    last_cav_pos = traci.vehicle.getPosition(last_cav_id)[0]
                    if abs(last_cav_pos - pos) <= R:
                        current_group.append(vid)
                    else:
                        self.window_vids.append(current_group)
                        current_group = []
                        current_group.append(vid)
                else:
                    current_group.append(vid)

            if len(current_group) == N:
                self.window_vids.append(current_group)
                current_group = []

        if current_group:
            self.window_vids.append(current_group)

    def get_state(self, vids, leader_id):
        '''
        根据vids获取车辆的状态, 其中vids包含该窗口内除
        '''
        state = [[int(leader_id[4]),
                  int(0),
                  round(traci.vehicle.getPosition(leader_id)[0], 2),
                  traci.vehicle.getLaneIndex(leader_id),
                  round(traci.vehicle.getSpeed(leader_id), 2)]]
        for id in vids:
            type = int(id[2])
            pos = round(traci.vehicle.getPosition(id)[0], 2)
            lane_index = traci.vehicle.getLaneIndex(id)
            speed = round(traci.vehicle.getSpeed(id), 2)
            s = [int(id[4]), type, pos, lane_index, speed]
            state.append(s)

        state.append([self.accident_lane, self.accident_pos])

        flattened_state = [item for sublist in state for item in sublist]

        return flattened_state

    def is_vid_in_join_state(self, vid):
        """
        判断vid是否在join_state里面，也就是是否执行了动作
        """
        # 遍历join_state中的每个键值对
        for key, value in self.join_state.items():
            if vid == key:
                return True
        return False

    def select_leader(self, vids):
        """
        从vids中获取leader
        """
        leader = 'v.0.0'

        # 先判断该window里面是否有leader, 有就不选
        set_vids = set(vids)
        set_leader_vids = set(self.leader_ids)
        if set_vids & set_leader_vids:
            print('该分组已经选择过leader了, leader为:', set_vids & set_leader_vids)
            return list(set_vids & set_leader_vids)[0]
        # 选择位置最靠前的CAV

        for id in vids:
            if id[2] == '0':
                # 确保 id 没有执行过加入车队的命令
                if not self.is_vid_in_join_state(id):
                    leader = id
                    break
        print('leader:', leader)

        if leader == 'v.0.0':
            print('该窗口内没有leader')
            return leader

        # leader 设置
        traci.vehicle.setColor(typeID=leader, color=((160, 32, 240)))

        self.plexe.set_path_cacc_parameters(vid=leader, distance=5, xi=2, omega_n=1, c1=0.5)
        self.plexe.set_fixed_lane(vid=leader, lane=self.platoon_lane, safe=True)

        traci.vehicle.setMaxSpeed(leader, MAX_SPEED)
        self.plexe.set_cc_desired_speed(leader, MAX_SPEED)
        self.plexe.set_acc_headway_time(leader, 1.5)

        self.plexe.use_controller_acceleration(leader, False)
        self.plexe.set_active_controller(vid=leader, controller=ACC)

        self.leader_ids.append(leader)

        return leader

    def choose_leader(self, vids):
        '''
        从vids中选择leader
        1) 位置最靠前的CAV -> 当前策略
        2) 在车队专用车道上且位置最靠前
        且每个窗口只选一次
        '''
        frist_vid = vids[0]
        print('frist_vid:', frist_vid)
        frist_vid_lane = traci.vehicle.getLaneID(frist_vid)[:2]
        # print('last_vid_lane:', last_vid_lane)
        if frist_vid_lane != 'E2':
            return
        set1 = set(vids)
        set2 = set(self.leader_ids)
        if set1 & set2:
            print('该分组已经选择过leader了, leader为:', set1 & set2)
            return
        cav_vids = [item for item in vids if item.startswith('v.0.')]
        # print(cav_vids)
        leader_id = None
        max_pos = 0
        for id in cav_vids:
            lane_index = traci.vehicle.getLaneIndex(id)
            if lane_index == self.platoon_lane:
                leader_id = id
                break
        if leader_id == None:
            leader_id = cav_vids[0]
        print('leader_id:', leader_id)

        # leader设置
        traci.vehicle.setColor(typeID = leader_id, color = ((160, 32, 240)))
        # traci.vehicle.setMaxSpeed(leader_id, MAX_SPEED)

        self.plexe.set_path_cacc_parameters(vid=leader_id, distance=5, xi=2, omega_n=1, c1=0.5)
        self.plexe.set_cc_desired_speed(leader_id, MAX_SPEED)
        self.plexe.set_acc_headway_time(leader_id, 1.5)

        self.plexe.use_controller_acceleration(leader_id, False)
        self.plexe.set_active_controller(vid=leader_id, controller=ACC)

        self.plexe.set_fixed_lane(vid = leader_id, lane = self.platoon_lane, safe = True)
        self.leader_ids.append(leader_id)

        return leader_id

    def perform_action(self, agent_vids, action, leader_id):
        '''
        执行动作
        '''
        # 将智能体执行动作的次序排好, 当同时执行动作时, 位置更靠前的智能体先执行动作
        action_list = []
        for i in range(len(agent_vids)):
            vid = agent_vids[i]
            pos = traci.vehicle.getPosition(vid)
            a = action[i]
            action_list.append([vid, a, pos[0]])
        # print(action_list)
        action_list = sorted(action_list, key=lambda x:x[-1], reverse=True)
        # print('action_list:', action_list)

        # change lane frist
        for a in action_list:
            if a[1] == 1:
                # perform action
                vid = a[0]
                traci.vehicle.setLaneChangeMode(vid, 512)
                lane = traci.vehicle.getLaneIndex(leader_id)
                if vid not in self.change_lane_state:
                    self.change_lane_state[vid] = {'state': 0, 'lane': lane}
                # traci.vehicle.changeLane(vid, lane, duration=5)

                if vid not in self.join_state:
                    self.join_state[vid] = {'state' : -1, 'front' : None, 'leader' : leader_id}

        action_list.clear()

        # 遍历动作列表
        # for a in action_list:
        #     if a[1] == 1:
        #         # leader减速
        #         # self.plexe.set_cc_desired_speed(leader_id, SPEED - 3)
        #         vid = a[0]
        #         fid = self.get_platoon_last_vid(leader_id)
        #         v_state = self.get_in_position(vid, fid, leader_id)
        #         self.join_state[vid] = {'state': v_state, 'front': fid, 'leader': leader_id}
        #         self.start_vids.append(vid)
        #
        # action_list.clear()

    def Is_change_lane(self):
        """
        检测变道是否完成, 必须先完成变道再执行之后的动作
        """
        if self.change_lane_state:
            for i in self.change_lane_state.keys():
                state = self.change_lane_state[i]['state']
                lane = self.change_lane_state[i]['lane']
                if state == 0:
                    # print(i, '未完成变道, 执行变道动作')
                    traci.vehicle.setLaneChangeMode(i, 512)
                    traci.vehicle.changeLane(i, lane, duration=5)
                    if traci.vehicle.getLaneIndex(i) == lane:
                        self.change_lane_state[i]['state'] = 1
                        print(i, '已完成变道')
                        self.join_state[i]['state'] = GOING_TO_POSITION
                        break

    def get_front_vid(self, vid):
        '''
        获取同车道前车vid
        '''
        lane_id = traci.vehicle.getLaneID(vid)

        vids = list(traci.lane.getLastStepVehicleIDs(lane_id))
        # 加上类型
        vids = [item for item in vids if item.startswith('v.0.')]
        index = vids.index(vid)
        # print('vid:', vid, 'front_id:' ,vids[index + 1])
        return vids[index + 1]

    def get_leader_vid(self, vid):
        """
        获取变道后, 前方车道的leader
        """
        lane_id = traci.vehicle.getLaneID(vid)

        vids = list(traci.lane.getLastStepVehicleIDs(lane_id))
        # 加上类型
        vids = [item for item in vids if item.startswith('v.0.')]
        index = vids.index(vid)

        for i in range(index, len(vids)):
            if vids[i] in self.leader_ids:
                return vids[i]

    def Is_joined_platoon(self):
        """
        检测执行动作的智能体CAV是否完成了合并操作,
        当距离合适时, 提示完成合并
        """
        if self.join_state:
            for vid in self.join_state.keys():
                v_state = self.join_state[vid]['state']
                leader_id = self.join_state[vid]['leader']
                if v_state == GOING_TO_POSITION:
                    fid = self.get_front_vid(vid)
                    self.join_state[vid]['front'] = fid

                    leader_id = self.get_leader_vid(vid)
                    self.join_state[vid]['leader'] = leader_id

                    traci.vehicle.setSpeedMode(vid, 0)
                    # traci.vehicle.setMaxSpeed(vid, MAX_SPEED)
                    # 加入拓扑结构
                    self.topology[vid] = {"leader": leader_id, "front": fid}
                    self.plexe.set_cc_desired_speed(vid, MAX_SPEED + 3)
                    self.plexe.set_active_controller(vid, FAKED_CACC)

                    if get_distance(self.plexe, vid, fid) < DISTANCE * 2:
                        print('距离够了完成合并')
                        self.plexe.set_path_cacc_parameters(vid, distance=5)
                        self.plexe.set_active_controller(vid, CACC)
                        self.join_state[vid]['state'] = COMPLETED
                        # 检测leader是否需要恢复速度
                        # if self.all_followers_joined(self.topology[i]['leader']):
                        #     self.plexe.set_cc_desired_speed(self.topology[i]['leader'], SPEED)
                        break

    def get_in_position(self, vid, fid, leader_id):
        '''
        控制vid追赶前车fid, 在加入leader所领导的车队
        '''
        # 设置车辆控制模式...具体详见...
        traci.vehicle.setSpeedMode(vid, 0)
        # traci.vehicle.setMaxSpeed(vid, MAX_SPEED)
        # 加入拓扑结构
        self.topology[vid] = {"leader": leader_id, "front": fid}
        self.plexe.set_cc_desired_speed(vid, MAX_SPEED + 3)
        self.plexe.set_active_controller(vid, FAKED_CACC)
        lane = traci.vehicle.getLaneIndex(leader_id)
        self.plexe.set_fixed_lane(vid, lane, safe=False)
        print('change the lane and get to the position')
        return GOING_TO_POSITION

    def change_lane(self, vid, leader):
        """
        CAV执行动作, 先变道
        """
        lane = traci.vehicle.getLaneIndex(leader)

        traci.vehicle.setLaneChangeMode(vid, 256)
        traci.vehicle.changeLane(vid, lane, duration=5)
        print(vid, ' changing lane to ', lane)

    def get_platoon_vids(self, leader_id):
        '''
        从拓扑结构中获取当前leader领导下的所有CAV的vids
        {
            'v.0.8':{'front':'v.0.9','leader':'v.0.9'}, 'v.0.7':{'front':'v.0.8','leader':'v.0.9'}
            'v.0.4':{'front':'v.0.6','leader':'v.0.6'}, 'v.0.5':{'front':'v.0.4','leader':'v.0.6'}, 'v.0.3':{'front':'v.0.5','leader':'v.0.6'}
            'v.0.1':{'front':'v.0.2','leader':'v.0.2'}
        }
        '''
        platoon_members = []

        # 构建每个leader领导的车队
        platoons = {}
        for vehicle, info in self.topology.items():
            leader = info['leader']
            if leader not in platoons:
                platoons[leader] = []
                platoons[leader].append(leader)
            platoons[leader].append(vehicle)

        #检查 leader_id 是否存在
        if leader_id in platoons:
            platoon_members = platoons[leader_id]

        return platoon_members

    def all_followers_joined(self, leader_id):
        '''
        判断以leader_id为领导的全部车辆是否全部都加入了车队
        '''
        followers = self.get_platoon_vids(leader_id)

        for follower_id in followers:
            if self.join_state.get(follower_id) and self.join_state[follower_id]['state'] != 2:
                return False

        return True

    def get_platoon_last_vid(self, leader_id):
        '''
        从拓扑结构中获取当前leader领导下的, 最后一辆CAV的vid
        '''
        platoon_members = self.get_platoon_vids(leader_id)
        # print(platoon_members)
        if platoon_members:
            return platoon_members[-1]
        else:
            return leader_id

    def get_window_cav_vids(self, vids):
        '''
        从window里面获取所要控制的agent的id
        还要去除窗口内所有的leader
        '''
        cav_vids = [item for item in vids if item.startswith('v.0.')]

        intersection = set(cav_vids) & set(self.leader_ids)

        cav_vids = [x for x in cav_vids if x not in intersection]

        # cav_vids.reverse()
        # if cav_vids not in self.window_cav_vids:
        #     self.window_cav_vids.append(cav_vids)
        return cav_vids

    def Is_leader_get_platoon_lane(self, leader_id):
        lane_index = traci.vehicle.getLaneIndex(leader_id)
        if lane_index == self.platoon_lane:
            return True
        else:
            return False

    def get_action(self, state, agent_ids):
        '''
        根据状态, 获取动作
        先根据状态判断输出动作的维度, 再调用模型
        '''
        action_size = len(agent_ids)
        print('action_size:', action_size)
        if np.random.rand() >= self.epsilon:
            print('---非随机动作')
            if action_size == 2:
                q = self.Q_network(np.array(state, dtype='float32').reshape([-1, 27]))
                a = np.argmax(q)
            # elif action_size == 3:
            #     q = ...
            #     a = np.argmax(q)
            # elif action_size == 4:
            #     q = ...
            #     a = np.argmax(q)
            # elif action_size == 1:
            #     q = ...
            #     a = np.argmax(q)
        else:
            a = np.random.randint(0, 2**action_size - 1)

        # print('a:', a)
        action = self.decimal_to_binary_list(a, num_bits = action_size)

        return action

    def marked_action(self, action, agent_ids):
        '''
        检查CAV的动作状态, 看是否需要对动作施加限制
        对于已经开始执行合并动作的CAV, 对其后的动作进行限制
        '''
        # print(self.join_state)
        for key, item in self.join_state.items():
            state_value = item['state']
            if state_value == GOING_TO_POSITION:
                # self.Is_joined_platoon()
                vid = key
                for i in range(len(agent_ids)):
                    if agent_ids[i] == vid:
                        action[i] = 0
            elif state_value == COMPLETED:
                vid = key
                for i in range(len(agent_ids)):
                    if agent_ids[i] == vid:
                        action[i] = 0
        return action

    def decimal_to_binary_list(self, decimal_num, num_bits):
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

    def process_data(self):
        """
        加载数据, 用于训练
        """
        data = random.sample(self.memory, self.batch)
        s = np.array([d[0] for d in data])
        a = [d[1] for d in data]
        s_ = np.array([d[2] for d in data])
        r = [d[3] for d in data]
        done = [d[4] for d in data]

        y = self.Q_network(np.array(s, dtype='float32'))
        y = y.numpy()
        Q1 = self.target_Q_network(np.array(s_, dtype='float32'))
        Q2 = self.Q_network(np.array(s_, dtype='float32'))
        next_action = np.argmax(Q2, axis=1)

        for i, (_, a, _, r, done) in enumerate(data):
            if done:
                target = r
            else:
                target = r + self.gamma * Q1[i][next_action[i]]
            target = np.array(target, dtype='float32')

            y[i][a] = target

        return s, y

    def update_Q_network(self, s, y):
        """
        更新Q网络, 返回损失loss
        """
        with tf.GradientTape() as tape:
            Q = self.Q_network(np.array(s, dtype='float32'))
            loss = tl.cost.mean_squared_error(Q, y)
        grads = tape.gradient(loss, self.Q_network.trainable_weights)
        self.opt.apply_gradients(zip(grads, self.Q_network.trainable_weights))

        return loss

    def update_target_Q(self):
        """
        更新目标网络
        """
        for i, target in zip(self.Q_network.trainable_weights, self.target_Q_network.trainable_weights):
            target.assign(i)

    def train(self):
        '''
        神经网络的训练
        '''
        s, y = self.process_data()
        loss = self.update_Q_network(s, y)
        self.total_loss.append(loss)
        if (self.episode + 1) % 20 == 0:
            self.update_target_Q()

    def get_avg_speed(self, vids):
        '''
        获取所有CAV的平均速度
        '''
        total_speed = 0
        for id in vids:
            total_speed += round(traci.vehicle.getSpeed(id), 2)
        avg_speed = total_speed / len(vids)
        return round(avg_speed, 2)

    def get_last_vid(self, vids):
        """
        获取最靠后vid的id以及位置
        """
        last_pos = None
        last_vid = None
        for id in vids:
            pos = traci.vehicle.getPosition(id)[0]
            if last_pos is None or pos < last_pos:
                last_vid = id
                last_pos = pos
        return last_vid, last_pos

    def get_reward(self, cav_vids):
        '''
        获取奖励值
        1) CAV的平均速度 speed
        2) 是否发生了碰撞
        3) 是否通过了瓶颈
        4) 是否完成了合并
        '''

        # r1 CAV average speed
        if len(cav_vids) == 0:
            r1 = 0
        else:
            r1 = self.get_avg_speed(cav_vids)

        # r2 whether 发生 collision
        if self.collision:
            r2 = COLLISION_REWARD
        else:
            r2 = 0

        # r3 whether pass the bottleneck area
        last_vid, last_pos = self.get_last_vid(cav_vids)
        if last_pos > self.accident_pos:
            print('The last CAV have passed the bottleneck area')
            r3 = 0
            pass_done = True
        else:
            r3 = TIME_REWARD
            pass_done = False

        # r4 is the ratio of CAVs perform platooning
        total_count = len(cav_vids)
        index_count = 0
        for cav in cav_vids:
            if cav in self.join_state:
                index_count += 1

        if total_count == 0:
            r4 = 0
        else:
            r4 = index_count / total_count

        if pass_done:
            r4 = 0
        # print('r4:', r4)

        if r4 == 1.0:
            r4 = 2

        reward = r1 + r2 + r4 * 40
        return reward

    def normalize_speed(self, speed):
        '''
        将速度标准化映射为[-1, 1]
        '''
        desire_speed = 30.55
        only_leader_speed = 31.48
        max_speed = 33.33
        speed_normaliazed = (speed - only_leader_speed) / (max_speed - only_leader_speed)

        return speed_normaliazed

    def get_reward_shaping(self, cav_vids, leader):
        '''
        获取标准化后的奖励
        '''
        # r0 -> the speed of CAVs passing bottleneck area
        cavs = cav_vids[:]
        cavs.append(leader)
        print('cav_vids: ', cavs)
        if self.done == True:
            avg_speed = self.get_avg_speed(cavs)
            r0 = self.normalize_speed(avg_speed)
            print('r0 = ', r0)
        else:
            r0 = 0

        # r1 -> safety
        if self.collision:
            r1 = COLLISION_REWARD
        else:
            r1 = 0
        # r2 -> platooning density
        total_count = len(cav_vids)
        index_count = 0
        for cav in cav_vids:
            if cav in self.join_state:
                index_count += 1

        if total_count == 0:
            r2 = 0
        else:
            r2 = index_count / total_count

        r = r0 + r1 + r2 * 0.1
        return r

    def remember(self, s, a, s_, r, done):
        """
        remember the data to experience replay
        """
        data = (s, a, s_, r, done)
        # print('data:', data)
        self.memory.append(data)
        # memory_str = '\n'.join(str(item) for item in self.memory)
        # out_file = 'output/output.txt'
        # with open(out_file, 'w') as file:
        #     file.write(memory_str)
        return self.memory

    def Is_done(self, window):
        '''
        该窗口是否完成 done
        1 -> 发生碰撞
        2 -> 最后一辆CAV通过瓶颈区域
        '''
        collision = [element for sublist in self.collision_vids for element in sublist]
        # 求交集
        intersection = list(set(collision).intersection(set(window)))
        # print('intersection:', intersection)
        if len(intersection) != 0:
            print('该窗口内已经发生碰撞')
            self.done = True
            return

        cav_ids = [item for item in window if item.startswith('v.0.')]
        print('cav_ids:', cav_ids)
        last_vid, last_pos = self.get_last_vid(cav_ids)
        print('last_vid:', last_vid, 'last_pos:', last_pos, 'accident_pos:', self.accident_pos)
        if last_pos > self.accident_pos:
            self.done = True

    def Is_pass_bottleneck(self, window):
        '''
        该窗口内最后一辆CAV是否已经通过瓶颈
        '''
        cav_ids = [item for item in window if item.startswith('v.0.')]
        print('cav_ids:', cav_ids)
        last_vid, last_pos = self.get_last_vid(cav_ids)
        print('last_vid:', last_vid, 'last_pos:', last_pos, 'accident_pos:', self.accident_pos)
        if last_pos > self.accident_pos:
            self.done = True

    def start_CAV_platoon(self, window):
        """
        判断是否开启车辆编队算法
        1) 当最后一辆CAV已经通过瓶颈区域后,就不再执行算法
        2) 窗口内的所有车都在编队区域
        3) 车辆速度不能太低
        """

        # cav_ids = [item for item in window if item.startswith('v.0.')]
        # print('cav_ids:', cav_ids)
        # last_vid, last_pos = self.get_last_vid(cav_ids)
        # print('last_vid:', last_vid, 'last_pos:', last_pos, 'accident_pos:', self.accident_pos)
        # if last_pos > self.accident_pos:
        #     return False

        total_speed = 0
        leader = window[0]
        leader_speed = round(traci.vehicle.getSpeed(leader), 2)
        for vid in window:
            lane = traci.vehicle.getLaneID(vid)[:2]
            speed = round(traci.vehicle.getSpeed(vid), 2)
            total_speed += speed
            # print('lane:', lane)
            if lane != self.start_platoon_lane:
                return False

        avg_speed = total_speed / len(window)
        # print('avg_speed:',avg_speed)
        if leader_speed > 29 or avg_speed > 28:
            return True
        else:
            return False

    def start_sumo(self):
        '''
        开启仿真, 测试代码
        '''
        while running(True, self.step, self.simulation_step):
            if self.step == self.simulation_step:
                print('仿真结束')
                start_sumo("cfg/highway.sumo.cfg", True)
                self.reinit()

            traci.simulationStep()

            if self.step == 0:
                traci.gui.trackVehicle("View #0", 'v.0.7')
                traci.gui.setZoom("View #0", 1500)

            if self.step == 300:
                self.get_total_vids()
                print('totoal_vids:', self.total_vids)

            if self.step == 200:
                self.create_accident()

            if self.step % 100 == 0:
                print('lane_pos:', traci.vehicle.getLanePosition('v.0.7'))
                print('pos:', traci.vehicle.getPosition('v.0.7'))

            # if self.step > 2000 and self.step % 500 == 0:
            #     windows = self.sliding_windows(N = 5, R = 100)
            #     print(windows)
            #     for window in windows:
            #         # 判断是否要执行编队算法
            #         if self.start_CAV_platoon(window):
            #             self.get_platoon_lane()
            #             leader = self.select_leader(window)
            #             if leader == 'v.0.0':
            #                 break
            #             cav_ids = self.get_window_cav_vids(window)
            #             action = []
            #             for i in range(len(cav_ids)):
            #                 action.append(1)
            #             marked_action = self.marked_action(action, cav_ids)
            #             print('leader:', leader, 'cav_ids:', cav_ids, 'action:', marked_action)
            #             if self.Is_leader_get_platoon_lane(leader):
            #                 self.perform_action(cav_ids, action, leader)



            # if self.step == 5000:
            #     self.get_platoon_lane()
            #
            # if self.step > 5000 and self.step % 200 == 0:
            #     self.clear_platoon_lane()
            #
            # if self.step > 6000 and self.step % 500 == 0:
            #     windows = self.sliding_windows(N=5, R=100)
            #     print(windows)
            #     for window in windows:
            #         leader = self.select_leader(window)
            #         if leader == 'v.0.0':
            #             break
            #         cav_ids = self.get_window_cav_vids(window)
            #         action = []
            #         for i in range(len(cav_ids)):
            #             action.append(1)
            #         marked_action = self.marked_action(action, cav_ids)
            #         print('leader:', leader, 'cav_ids:', cav_ids, 'action:', marked_action)
            #         if self.Is_leader_get_platoon_lane(leader):
            #             self.perform_action(cav_ids, action, leader)



            # if self.step % 1000 == 0 and self.step > 0:
            #     windows = self.sliding_windows(N=5,R=50)
            #     print(windows)
            #     for window in windows:
            #         self.select_leader(window)

                # self.get_vids_in_rectangle(N=5,R=100)
                # print('window_t_vids:', self.window_vids)
                # if self.window_vids:
                #     self.get_platoon_lane()
                #     self.clear_platoon_lane()
                #     for window_vid in self.window_vids:
                #         self.choose_leader(window_vid)
                #         self.get_window_cav_vids(window_vid)
                # print('window_total_cav_vids:', self.window_cav_vids)

            # if self.step % 800 == 0 and self.step > 6000:
            #     if self.window_cav_vids:
            #         print('已经开始车辆编队, 执行动作')
            #         for i in range(len(self.window_vids)):
            #             vids = self.window_vids[i][:]
            #             cav_vids = self.window_cav_vids[i]
            #             # print('vids:',vids,'cav_vids:',cav_vids)
            #             leader_id = self.leader_ids[i]
            #             if leader_id in vids:
            #                 vids.remove(leader_id)
            #             agent_vids = cav_vids[:]
            #             if leader_id in agent_vids:
            #                 agent_vids.remove(leader_id)
            #             action = []
            #             for i in range(len(agent_vids)):
            #                 action.append(1)
            #             marked_action = self.marked_action(action, agent_vids)
            #             print('perform_action:', leader_id, agent_vids, marked_action)
            #             # state = self.get_state(vids, leader_id)
            #             # print('state: ', state)
            #             # print(len(state))
            #             if self.Is_leader_get_platoon_lane(leader_id):
            #                 self.perform_action(agent_vids, marked_action, leader_id)

            # if self.step % 100 == 0:
            #     self.vsl()
            #     self.Is_joined_platoon()
            #     print('join_state:', self.join_state)
            #     if self.platoon_lane >= 0:
            #         self.clear_platoon_lane()

            # if self.step % 100 == 1:
            #     communicate(self.plexe, self.topology)



            self.step += 1

        traci.close()

    def start_train(self):
        """
        开始训练, 按不同CAV渗透率的组进行训练
        """
        while running(True, self.step, self.simulation_step):
            if self.step == self.simulation_step:
                self.episode = self.episode + 1
                # 记录数据
                print('episode: ', self.episode, '; total_reward: ', self.total_reward)
                print('------第', self.episode, '回合仿真结束------')
                with open(self.file_path + '_total_reward.txt', 'a') as file:
                    file.write(f"Episode {self.episode}: Total Reward = {self.total_reward}\n")

                print('join_state:', self.join_state)

                if (self.episode + 1) % 1000 == 0 or self.episode == 1:
                    print('保存模型')
                    self.Q_network.save(filepath= self.file_path +'_' + str(self.episode) + '_model.h5', save_weights = True)


                if len(self.memory) > self.batch and self.episode > 800:
                    self.train()

                if self.episode >= self.episode_max:
                    print('-----训练结束-----')
                    break

                self.reinit()
                if (self.episode + 1) % 20 == 0 and self.episode > 300:
                    self.update_epsilon()
                    print('epsiode:',self.episode,'epsilon:', self.epsilon)

                # 重新开启下回合的训练
                start_sumo("cfg/highway.sumo.cfg", True, gui=self.is_rend)

            traci.simulationStep()

            colliding_vehicles = list(traci.simulation.getCollidingVehiclesIDList())
            if len(colliding_vehicles) != 0:
                print('发生碰撞,车辆ID为:', colliding_vehicles)
                self.collision_vids.append(colliding_vehicles)
                self.collision = True

            if self.step == 0 and self.is_rend:
                random_vid = traci.vehicle.getIDList()[0]
                traci.gui.trackVehicle("View #0", random_vid)
                traci.gui.setZoom("View #0", 1500)

            if self.step == 200:
                self.create_accident()

            if self.step > 2000 and self.step % 300 == 0:
                windows = self.sliding_windows(N = 5, R = 100000)
                print('windows:', windows)
                for window in windows:
                    # 判断是否要开始执行编队算法
                    if self.start_CAV_platoon(window) and not self.done:
                        print('执行车辆编队算法')
                        self.get_platoon_lane()
                        leader = self.select_leader(window)
                        if leader == 'v.0.0':
                            break
                        if not self.Is_leader_get_platoon_lane(leader):
                            break
                        vids = window[::]
                        vids.remove(leader)
                        cav_ids = self.get_window_cav_vids(window)

                        # remember data (s,a,s',r,done)
                        if self.frist_action:
                            self.frist_action = False
                        else:
                            next_state = self.get_state(vids, leader)
                            self.Is_done(window)
                            # self.Is_pass_bottleneck(window)
                            # reward = self.get_reward(cav_ids)
                            reward = self.get_reward_shaping(cav_ids, leader)
                            print('reward: ', reward)
                            self.total_reward += reward
                            self.memory = self.remember(state, action, next_state, reward, self.done)
                            print('done:', self.done)
                            if self.done:
                                continue
                        # get state
                        state = self.get_state(vids, leader)
                        print('state:', state)
                        # print('state_len:', len(state))
                        # get action
                        if self.episode < 300:
                            action = []
                            for i in range(len(cav_ids)):
                                action.append(1)
                        else:
                            action = self.get_action(state, cav_ids)

                        # marked action
                        marked_action = self.marked_action(action, cav_ids)

                        # perfrom action
                        print('leader:', leader, 'cav_ids:', cav_ids, 'action:', marked_action)
                        self.perform_action(cav_ids, marked_action, leader)

            if self.step % 50 == 0:
                self.vsl()
                self.Is_joined_platoon()
                self.Is_change_lane()
                if self.platoon_lane >= 0:
                    self.clear_platoon_lane()
                communicate(self.plexe, self.topology)
                # print('join_state:', self.join_state)
                # print('collision_vids:', self.collision_vids)

            self.step += 1

        traci.close()

    def test_model(self, test_episode):
        '''
        测试模型
        '''
        self.Q_network.load(filepath= 'output/23_10_25_3999_model.h5', load_weights = True)
        self.episode_max = test_episode
        while running(True, self.step, self.simulation_step):
            if self.step == self.simulation_step:
                self.episode = self.episode + 1
                # 记录数据
                print('episode: ', self.episode, '; total_reward: ', self.total_reward)
                print('------第', self.episode, '回合仿真结束------')
                with open(self.file_path + '_test_total_reward.txt', 'a') as file:
                    file.write(f"Episode {self.episode}: Total Reward = {self.total_reward}\n")

                print('join_state:', self.join_state)

                if self.episode >= self.episode_max:
                    print('-----测试结束-----')
                    break

                self.reinit()

                # 重新开启下回合的训练
                start_sumo("cfg/highway.sumo.cfg", True, gui=self.is_rend)

            traci.simulationStep()

            colliding_vehicles = traci.simulation.getCollidingVehiclesIDList()
            if len(colliding_vehicles) != 0:
                print('发生碰撞,车辆ID为:', colliding_vehicles)
                self.collision = True

            if self.step == 0 and self.is_rend:
                random_vid = traci.vehicle.getIDList()[0]
                traci.gui.trackVehicle("View #0", random_vid)
                traci.gui.setZoom("View #0", 1500)

            if self.step == 200:
                self.create_accident()

            if self.step > 2000 and self.step % 300 == 0:
                windows = self.sliding_windows(N = 5, R = 100000)
                print('windows:', windows)
                for window in windows:
                    # 判断是否要开始执行编队算法
                    if self.start_CAV_platoon(window) and not self.done:
                        print('执行车辆编队算法')
                        self.get_platoon_lane()
                        leader = self.select_leader(window)
                        if leader == 'v.0.0':
                            break
                        if not self.Is_leader_get_platoon_lane(leader):
                            break
                        vids = window[::]
                        vids.remove(leader)
                        cav_ids = self.get_window_cav_vids(window)

                        # remember data (s,a,s',r,done)
                        if self.frist_action:
                            self.frist_action = False
                        else:
                            reward = self.get_reward(cav_ids)
                            self.Is_pass_bottleneck(window)
                            self.total_reward += reward
                            print('done:', self.done)
                            if self.done:
                                continue
                        # get state
                        state = self.get_state(vids, leader)
                        print('state:', state)
                        # print('state_len:', len(state))

                        # get action
                        q = self.Q_network(np.array(state, dtype='float32').reshape([-1, 27]))
                        a = np.argmax(q)
                        action = self.decimal_to_binary_list(a, 2)
                        print('action:', action)

                        # marked action
                        marked_action = self.marked_action(action, cav_ids)

                        # perfrom action
                        print('leader:', leader, 'cav_ids:', cav_ids, 'marked_action:', marked_action)
                        self.perform_action(cav_ids, marked_action, leader)


            if self.step % 50 == 0:
                self.vsl()
                self.Is_joined_platoon()
                self.Is_change_lane()
                if self.platoon_lane >= 0:
                    self.clear_platoon_lane()
                communicate(self.plexe, self.topology)
                # print('join_state:', self.join_state)

            self.step += 1

        traci.close()

    def start_large_test(self):
        """
        大规模交通流测试
        """
        while running(True, self.step, self.simulation_step):
            if self.step == self.simulation_step:
                self.episode = self.episode + 1
                # 记录数据
                print('episode: ', self.episode, '; total_reward: ', self.total_reward)
                print('------第', self.episode, '回合仿真结束------')
                with open(self.file_path + '_large_test_total_reward.txt', 'a') as file:
                    file.write(f"Episode {self.episode}: Total Reward = {self.total_reward}\n")

                print('join_state:', self.join_state)

                if self.episode >= self.episode_max:
                    print('-----测试结束-----')
                    break

                self.reinit()

                # 重新开启下回合的训练
                start_sumo("cfg/highway.sumo.cfg", True, gui=self.is_rend)

            traci.simulationStep()

            colliding_vehicles = list(traci.simulation.getCollidingVehiclesIDList())
            if len(colliding_vehicles) != 0:
                print('发生碰撞,车辆ID为:', colliding_vehicles)
                self.collision_vids.append(colliding_vehicles)
                self.collision = True

            if self.step %  2000 ==  0 and self.step > 0:
                self.create_vehicles()
                # self.create_vehicles_random()


            if self.step == 10 and self.is_rend:
                random_vid = traci.vehicle.getIDList()[0]
                traci.gui.trackVehicle("View #0", random_vid)
                traci.gui.setZoom("View #0", 1500)



            if self.step == 200:
                self.create_accident()

            if self.step > 2000 and self.step % 300 == 0:
                windows = self.sliding_windows(N = 5, R = 100000)
                print('windows:', windows)
                for window in windows:
                    # 判断是否要开始执行编队算法
                    if self.start_CAV_platoon(window) and not self.done:
                        print('执行车辆编队算法')
                        self.get_platoon_lane()
                        leader = self.select_leader(window)
                        if leader == 'v.0.0':
                            break
                        if not self.Is_leader_get_platoon_lane(leader):
                            break
                        vids = window[::]
                        vids.remove(leader)
                        cav_ids = self.get_window_cav_vids(window)

                        # get state
                        state = self.get_state(vids, leader)
                        print('state:', state)
                        # print('state_len:', len(state))
                        # get action
                        action = []
                        if self.episode < 300:
                            for i in range(len(cav_ids)):
                                action.append(1)
                        else:
                            action = self.get_action(state, cav_ids)

                        # marked action
                        marked_action = self.marked_action(action, cav_ids)

                        # perfrom action
                        print('leader:', leader, 'cav_ids:', cav_ids, 'action:', marked_action)
                        self.perform_action(cav_ids, marked_action, leader)

            if self.step % 50 == 0:
                self.vsl()
                self.Is_joined_platoon()
                self.Is_change_lane()
                if self.platoon_lane >= 0:
                    self.clear_platoon_lane()
                communicate(self.plexe, self.topology)
                print('join_state:', self.join_state)
                print('change_lane_state:', self.change_lane_state)
                # print('collision_vids:', self.collision_vids)

            self.step += 1

        traci.close()

    def test_change_lane(self):
        """
        大规模交通流测试
        """
        while running(True, self.step, self.simulation_step):
            if self.step == self.simulation_step:
                self.episode = self.episode + 1
                # 记录数据
                print('episode: ', self.episode, '; total_reward: ', self.total_reward)
                print('------第', self.episode, '回合仿真结束------')
                with open(self.file_path + '_large_test_total_reward.txt', 'a') as file:
                    file.write(f"Episode {self.episode}: Total Reward = {self.total_reward}\n")

                print('join_state:', self.join_state)

                if self.episode >= self.episode_max:
                    print('-----测试结束-----')
                    break

                self.reinit()

                # 重新开启下回合的训练
                start_sumo("cfg/highway.sumo.cfg", True, gui=self.is_rend)

            traci.simulationStep()

            colliding_vehicles = list(traci.simulation.getCollidingVehiclesIDList())
            if len(colliding_vehicles) != 0:
                print('发生碰撞,车辆ID为:', colliding_vehicles)
                self.collision_vids.append(colliding_vehicles)
                self.collision = True

            if self.step == 0:
                # self.create_vehicles()
                self.create_vehicles_random()


            if self.step == 10 and self.is_rend:
                random_vid = traci.vehicle.getIDList()[0]
                traci.gui.trackVehicle("View #0", random_vid)
                traci.gui.setZoom("View #0", 1500)

            if self.step == 200:
                self.create_accident()

            if self.step > 2000 and self.step % 300 == 0:
                windows = self.sliding_windows(N = 5, R = 100000)
                print('windows:', windows)
                for window in windows:
                    # 判断是否要开始执行编队算法
                    if self.start_CAV_platoon(window) and not self.done:
                        print('执行车辆编队算法')
                        self.get_platoon_lane()
                        leader = self.select_leader(window)
                        if leader == 'v.0.0':
                            break
                        if not self.Is_leader_get_platoon_lane(leader):
                            break
                        vids = window[::]
                        vids.remove(leader)
                        cav_ids = self.get_window_cav_vids(window)

                        # perform action
                        action = []
                        if self.episode < 300:
                            for i in range(len(cav_ids)):
                                action.append(1)

                        action = self.marked_action(action, cav_ids)

                        self.perform_action(cav_ids, action, leader)



            if self.step % 50 == 0:
                self.vsl()
                self.Is_joined_platoon()
                self.Is_change_lane()
                if self.platoon_lane >= 0:
                    self.clear_platoon_lane()
                communicate(self.plexe, self.topology)
                print('join_state:', self.join_state)
                # print('change_lane_state:', self.change_lane_state)
                # print('collision_vids:', self.collision_vids)

            self.step += 1

        traci.close()


if __name__ == "__main__":
    Highway = Highway()
    Highway.start_sumo()
    # Highway.start_train()
    # Highway.test_model(5)
    # Highway.start_large_test()
    # Highway.test_change_lane()
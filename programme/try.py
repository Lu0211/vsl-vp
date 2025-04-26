# #!/usr/bin/env python
# # -*- coding: UTF-8 -*-
# '''
# @Project ：DQN.py
# @File    ：try.py
# @Author  ：Lu
# @Date    ：2023/5/1 19:27
# '''
# import random
#
# import numpy as np
# import tensorflow as tf
# import tensorlayer as tl
#
# def get_model():
#     # DQN
#     input = tl.layers.Input(shape = [4,4])
#     h0 = tl.layers.Reshape(shape=(-1,16))(input)
#     h1 = tl.layers.Dense(32, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(h0)
#     h2 = tl.layers.Dense(16, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(h1)
#     output = tl.layers.Dense(6, act=tf.nn.sigmoid, W_init=tf.initializers.GlorotUniform())(h2)
#     reshape_out = tl.layers.Reshape((3, 2))(output)
#     return tl.models.Model(inputs=input, outputs=reshape_out)
#
#     # dueling DQN只改了网络架构。
# # def get_model(input_dim, output_dim):
# #     # 第一部分
# #     input = tl.layers.Input(shape=[None, input_dim])
# #     h1 = tl.layers.Dense(16, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(input)
# #     h2 = tl.layers.Dense(16, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(h1)
# #     # 第二部分
# #     svalue = tl.layers.Dense(output_dim, act=tf.nn.sigmoid)(h2)
# #     # 第三部分
# #     avalue = tl.layers.Dense(output_dim, act=tf.nn.sigmoid)(h2)  # 计算avalue
# #     mean = tl.layers.Lambda(lambda x: tf.reduce_mean(x, axis=1, keepdims=True))(avalue)  # 用Lambda层，计算avg(a)
# #     advantage = tl.layers.ElementwiseLambda(lambda x, y: x - y)([avalue, mean])  # a - avg(a)
# #
# #     output = tl.layers.ElementwiseLambda(lambda x, y: x + y)([svalue, avalue])
# #     return tl.models.Model(inputs=input, outputs=output)
#
# import numpy
# state = [['3', 171.84946823246992, 0, 28.354119301173387],
#          ['1', 133.47601763596913, 2, 29.49005768977149],
#          ['2', 148.1952523845804, 1, 29.43653475796378],
#          ['1', 1000.0, 2, 0]]
# s = numpy.array(state, dtype='float32')
# a,b = s.shape
# print('shape:', s.shape)
# Network = get_model()
# Network.train()
#
#
# def get_action(state, Q_network, epsilon = 0):
#     if np.random.rand() >= epsilon:
#         print('---非随机---')
#         q = Q_network(np.array(state, dtype='float32'))
#         a = np.argmax(np.array(q), axis=1)
#         return a
#     # 否则，随机一个动作输出。
#     else:
#         a = np.random.randint(2, size=3)
#         return a
#
#
# action = get_action(state, Network)
# print(action)
#
# vids = s[1:-1, 0]
# print("vids:", vids)
# vid = int(vids[0])
# print(vid)
#
# action_list = [['v.0.1', 500, 0], ['v.0.2', 500, 0], ['v.0.1', 1000, 1], ['v.0.2', 1000, 0]]
# for a in action_list:
#     if a[2] == 1:
#         vid = a[0]
#         begin_step = a[1]
#         print(vid, begin_step, '---')

import random


def generate_random_list(n):
    'random list: length = n'
    min_interval = 15
    max_interval = 45

    # first number
    list = [random.randint(1, 10)]

    for i in range(1, n):
        interval = random.randint(min_interval, max_interval)
        next_number = list[-1] + interval
        list.append(next_number)

    return list


def get_random_vehicle_data(n):
    "随机生成车流 1) random number of numbers in each lane; 2) random pos; 3) random type"

    # 每条车道上的车辆数 lane_0 lane_1 lane_2
    n_lane = [int(n / 3), int(n / 3), n - int(n / 3) - int(n / 3)]
    random.shuffle(n_lane)
    print(n_lane)
    # random type
    n_cav = 9
    n_hdv = 8
    n_truck = 3
    type_list = [0] * n_cav + [1] * n_hdv + [3] * n_truck
    random.shuffle(type_list)
    print(type_list)
    lane_0_type = type_list[:n_lane[0]]
    lane_1_type = type_list[n_lane[0]: n_lane[0] + n_lane[1]]
    lane_2_type = type_list[n_lane[0] + n_lane[1]:]
    print(lane_0_type, lane_1_type, lane_2_type)

    # lane_0
    pos_0 = generate_random_list(n_lane[0])
    print('lane 0:', pos_0, lane_0_type)

    # lane_1
    pos_1 = generate_random_list(n_lane[1])
    print('lane 1:', pos_1, lane_1_type)

    # lane_2
    pos_2 = generate_random_list(n_lane[2])
    print('lane 2:', pos_2, lane_2_type)

vehicle_data = get_random_vehicle_data(20)
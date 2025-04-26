#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：scenario_highway.py 
@File    ：try4.py
@Author  ：Lu
@Date    ：2023/10/17 20:08 
'''

## get epsilon train change
# epsilon = 1.0
# decay = 0.95
# min = 0.01
# n = 0
# while(epsilon >= min):
#     n += 1
#     if n % 20 == 0 and n > 300:
#         epsilon *= decay
#         print('n:', n, ' epsilon:', epsilon)

import tensorflow as tf
import tensorlayer as tl
import numpy as np

# 保留两位小数
def get_model(input_dim, output_dim):
    "获取模型"
    # DQN
    input = tl.layers.Input(shape=(None, input_dim))
    h1 = tl.layers.Dense(64, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(input)
    h2 = tl.layers.Dense(32, tf.nn.relu, W_init=tf.initializers.GlorotUniform())(h1)
    output = tl.layers.Dense(output_dim, act=None, W_init=tf.initializers.GlorotUniform())(h2)

    return tl.models.Model(inputs=input, outputs=output)

Q_network = get_model(input_dim=27, output_dim=4)
Q_network.train()

state = [1, 0, 2365.06, 0, 33.33, 3, 0, 2449.43, 0, 33.51, 6, 1, 2422.52, 1, 22.59, 4, 1, 2379.26, 1, 25.76, 2, 0, 2354.83, 0, 33.52, '1', 2500.0]

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

q = Q_network(np.array(state, dtype='float32').reshape([-1, 27]))
a = np.argmax(q)
print('q:', q)
print('a:', a)
action = decimal_to_binary_list(a, 2)
print('action:', action)



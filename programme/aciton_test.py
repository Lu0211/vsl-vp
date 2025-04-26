#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：functions.py 
@File    ：aciton_test.py
@Author  ：Lu
@Date    ：2023/8/11 11:07
'''
import numpy as np
import tensorflow as tf
import tensorlayer as tl

def get_action(state, Q_network, epsilon):
    '''
    获取动作, 按照epsilon探索策略
    :param state:
    :param Q_network:
    :return: action(list)
    '''
    if np.random.rand() >= epsilon:
        print('---非随机---')
        q = Q_network(np.array(state, dtype='float32'))
        a = np.argmax(np.array(q), axis=1)
        return a.tolist()
    # 否则，随机一个动作输出。
    else:
        a = np.random.randint(0, 3)
        return a.tolist()

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

Q_network = get_model()
Q_network.train()
state = [3, 1700.1378137149948, 0, 30.55000000000009, 1, 1681.8941541124614, 0, 30.586656335127707, 2, 1691.0793040711822, 0, 30.559131799855106, '1', 1000.0]
q = Q_network(np.array(state, dtype='float32').reshape([-1, 14]))
print(q)
action = np.argmax(q)
print(action)
a = decimal_to_binary_list(action, 2)
print(a)
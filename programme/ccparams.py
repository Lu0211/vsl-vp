#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：programme.py
@File    ：ccparams.py
@Author  ：Lu
@Date    ：2023/4/12 10:24 
'''

# vehicle length
LENGTH = 4

# inter-vehicle distance
DISTANCE = 5

# cruising speed 120 km/h = 33.3 m/s
DESIRED_SPEED = 30.55
SPEED = 33.33
MAX_SPEED = 33.33

# distance between multiple platoons
PLATOON_DISTANCE = MAX_SPEED * 1.5 + 2

# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2

# maneuver states:
GOING_TO_POSITION = 0  # joiner going to the position
OPENING_GAP = 1  # joiner arrive at the position and waiting for the opening gap
COMPLETED = 2  # joiner joining the platoon

# collision
COLLISION = False

# whether begin join
JOIN = False

# 窗口内允许最大车辆数
MAX_R_NUM = 6
# 窗口最大长度
MAX_R_LENGTH = 800

# 碰撞时的负奖励
COLLISION_REWARD = -100
# 该时刻未通过瓶颈区域的负奖励
TIME_REWARD = -50
# 该时刻未完成合并的负奖励
JOIN_REWARD = -20

# epsilon衰退率
EPSILON_DECAY = 0.98
# epsilonz最小值
EPSILON_MIN = 0.01
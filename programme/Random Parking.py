#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：scenario_2.py 
@File    ：环境随机性.py
@Author  ：Lu
@Date    ：2023/5/26 10:19 
'''
'''
环境随机性的体现:
1) 车辆速度位置随机
2) 瓶颈大小位置随机
3) 车辆类型随机
'''
## Initialization module

import os
import sys
import plexe
import random
import numpy as np
# 调用 utils module，里面包含了 platooning 的中层实现函数
from utils import add_platooning_vehicle, start_sumo, running, communicate, add_hv_vehicle, get_distance
# 确保路径设置正确，python 能够搜索到 traci module
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import random
import math
def get_random_numbers(v_num):
    # 每隔20个数选择一个随机数, 而且保证每两个数之间的差值要大于10
    numbers = []
    lower_limit = 1
    for i in range(1, v_num+1):
        upper_limit = lower_limit + 9 + (i - 1) * 10
        number = random.randint(lower_limit, upper_limit)
        numbers.append(number)
        lower_limit = number + 11  # 为了确保任何两个随机数之间的差大于10，我们在每次生成随机数后，将下限设置为当前随机数加上11。
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
            traci.vehicle.add(vehID=vid, routeID="platoon_route", typeID="vtypeauto",
                          departPos=str(vPos), departLane=str(vLane), departSpeed=str(random.randint(20,25)))
        elif vType == 1:
            if random.random() >= 0.5:
                type = "passenger"
            else:
                type = "passenger2"
            traci.vehicle.add(vehID=vid, routeID="platoon_route", typeID=type,
                              departPos=str(vPos), departLane=str(vLane), departSpeed=str(random.randint(20, 25)))


create_vehicle(5)
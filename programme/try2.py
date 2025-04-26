#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：scenario_5.py 
@File    ：try2.py
@Author  ：Lu
@Date    ：2023/7/4 17:23 
'''
import numpy

from ccparams import GOING_TO_POSITION, COMPLETED
topology = {'v.0.2': {}}
topology2 = {'v.0.2': {}, 'v.0.1': {'leader': 'v.0.2', 'front': 'v.0.2'}}
topology3 = {'v.0.2': {}, 'v.0.1': {'leader': 'v.0.2', 'front': 'v.0.2'},
             'v.0.0': {'leader': 'v.0.2', 'front': 'v.0.1'}}
vid = 'v.0.2'
join_state = {'v.0.2': {'state': 0, 'front_id': 'v.0.3'}, 'v.0.1': {'state': 0, 'front_id': 'v.0.2'}}
# topology = Join_platoon(plexe, join_id='v.0.1', leader_id=leader_id,
#                         front_id=leader_id, topology=topology, step=step, begin_join=400)

agent_ids = ['1','2']
action = [1, 1]
for key, item in join_state.items():
    state_value = item['state']
    if state_value == GOING_TO_POSITION:
        vid = int(key[4])
        print(vid)
        for i in range(len(agent_ids)):
            if int(agent_ids[i]) == vid:
                action[i] = 0
# print(action)

import json

# 从文件中读取JSON数据
with open('data.json', 'r') as file:
    loaded_data = json.load(file)


print(loaded_data[1])



def add_data_to_json(file, data):
    # 从文件中读取JSON数据
    with open('data.json', 'r') as file:
        loaded_data = json.load(file)

    last_list = loaded_data[-1]
    last_list.append(data)

    with open('data.json', 'w') as file:
        json.dump(loaded_data, file)



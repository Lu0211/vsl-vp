#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：DQN.py 
@File    ：1.py
@IDE     ：PyCharm 
@Author  ：Lu
@Date    ：2023/8/2 16:21 
'''
def get_team_members(topology, leader_id):
    team_members = []  # 存储由指定 leader_id 领导的车队中的所有车辆ID

    # 构建每个leader领导的车队
    teams = {}
    for vehicle, info in topology.items():
        leader = info['leader']
        if leader not in teams:
            teams[leader] = []
            teams[leader].append(leader)
        teams[leader].append(vehicle)

    print(teams)
    # 检查 leader_id 是否存在
    if leader_id in teams:
        team_members = teams[leader_id]

    return team_members

# 示例用法

topology = {
            'v.0.8':{'front':'v.0.9','leader':'v.0.9'},
            'v.0.7':{'front':'v.0.8','leader':'v.0.9'},
            'v.0.4':{'front':'v.0.6','leader':'v.0.6'},
            'v.0.5':{'front':'v.0.4','leader':'v.0.6'},
            'v.0.3':{'front':'v.0.5','leader':'v.0.6'},
            'v.0.1':{'front':'v.0.2','leader':'v.0.2'}
        }

leader_id = 'v.0.6'
team_members = get_team_members(topology, leader_id)
print(team_members[-1])

if team_members:
    print(f"由 {leader_id} 领导的车队中的所有车辆的ID如下:")
    for i, member in enumerate(team_members):
        print(f"{i+1}: {member}")
else:
    print(f"未找到由 {leader_id} 领导的车队")




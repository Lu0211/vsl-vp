#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：train.py 
@File    ：try3.py
@Author  ：Lu
@Date    ：2023/8/24 21:50 
'''

vids = ['v.0.9', 'v.1.6', 'v.0.8', 'v.1.5', 'v.0.7', 'v.1.4', 'v.0.5', 'v.0.6', 'v.0.4', 'v.1.3', 'v.0.3', 'v.1.2', 'v.0.1', 'v.1.1', 'v.0.2']
pos = [150.76037586657998, 142.41212052971972, 125.72690704840502, 120.32518428533939, 108.75887669012145, 101.62293484140845, 84.73400016240467, 80.76603458176936, 73.5872287660054, 56.81772388771526, 50.75776278054066, 49.80121081217039, 30.407057952715373, 24.15309335779426, 23.39384041063824]
# 修改后的分组函数
def group_vehicles(vids, positions, N, R):
    groups = []
    current_group = []

    for vid, pos in zip(vids, positions):
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
                    groups.append(current_group)
                    current_group = []
                    current_group.append(vid)
            else:
                current_group.append(vid)

        if len(current_group) == N:
            groups.append(current_group)
            current_group = []

    if current_group:
        groups.append(current_group)

    return groups

# 判断车辆是否为CAV
def is_cav(vehicle_type):
    return vehicle_type == 'v.0'

# 检查通信距离
def is_within_communication_distance(position1, position2, R):
    distance = abs(position1 - position2)
    return distance <= R

# 调用分组函数
max_group_size = 5  # 每个分组的最大车辆数
R = 100  # 通信距离
grouped_vehicles = group_vehicles(vids[::-1], pos, max_group_size, R)

# 打印分组结果
for i, group in enumerate(grouped_vehicles):
    print(f"Group {i + 1}: {group}")


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

            if self.step > 2000 and self.step % 500 == 0:
                windows = self.sliding_windows(N = 5, R = 100)
                print(windows)
                for window in windows:
                    # 判断是否要执行编队算法
                    if self.start_CAV_platoon(window):
                        self.get_platoon_lane()
                        leader = self.select_leader(window)
                        if leader == 'v.0.0':
                            break
                        cav_ids = self.get_window_cav_vids(window)
                        action = []
                        for i in range(len(cav_ids)):
                            action.append(1)
                        marked_action = self.marked_action(action, cav_ids)
                        print('leader:', leader, 'cav_ids:', cav_ids, 'action:', marked_action)
                        if self.Is_leader_get_platoon_lane(leader):
                            self.perform_action(cav_ids, marked_action, leader)



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

            if self.step % 100 == 0:
                self.vsl()
                self.Is_joined_platoon()
                print('join_state:', self.join_state)
                if self.platoon_lane >= 0:
                    self.clear_platoon_lane()

            if self.step % 100 == 1:
                communicate(self.plexe, self.topology)
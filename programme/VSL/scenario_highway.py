#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：train.py 
@File    ：scenario_highway.py
@Author  ：Lu
@Date    ：2023/8/24 10:45 
'''

import traci
import os
import plexe
import random
from utils import add_platooning_vehicle, start_sumo, running, communicate, add_hv_vehicle, get_distance
from functions import create_vehicle

def create_vehs(max_cav_id, max_hdv_id):
    vids = []
    for i in range(max_cav_id, max_cav_id + 3):
        id = 'v.0.%d' % (i + 1)
        vids.append(id)
    for j in range(max_hdv_id, max_hdv_id + 2):
        id = 'v.1.%d' % (j + 1)
        vids.append(id)
    print(vids)
    pos = [10, 25, 50, 25 ,55]
    lane = [2, 1, 0, 0, 2]
    for k in range(len(vids)):
        if vids[k][2] == '0':
            traci.vehicle.add(vehID=vids[k], routeID="vehicles_route", typeID="vtypeauto",
                          departPos=str(pos[k]), departLane=str(lane[k]), departSpeed=str(15))
        else:
            traci.vehicle.add(vehID=vids[k], routeID="vehicles_route", typeID="passenger",
                              departPos=str(pos[k]), departLane=str(lane[k]), departSpeed=str(15))

def create_init_vehs():
    cav_ids = ['v.0.1','v.0.2','v.0.3','v.0.4','v.0.5','v.0.6','v.0.7','v.0.8','v.0.9']
    cav_pos = [10, 3, 30, 40, 48, 52, 60, 65, 80]
    cav_lane_id = [1, 2, 0, 2, 0, 1, 1, 2, 0]
    hdv_ids = ['v.1.1', 'v.1.2', 'v.1.3', 'v.1.4', 'v.1.5', 'v.1.6']
    hdv_pos = [11, 20, 42, 50, 58, 75]
    hdv_lane_id = [0, 2, 1, 2, 0, 2]
    for i in range(len(cav_ids)):
        traci.vehicle.add(vehID=cav_ids[i], routeID="vehicles_route", typeID="vtypeauto",
                          departPos=str(cav_pos[i]), departLane=str(cav_lane_id[i]), departSpeed=str(15))
    for j in range(len(hdv_ids)):
        traci.vehicle.add(vehID=hdv_ids[j], routeID="vehicles_route", typeID="passenger",
                          departPos=str(hdv_pos[j]), departLane=str(hdv_lane_id[j]), departSpeed=str(15))

def get_last_id_from_vids(vids):
    max_cav_id = -1
    max_hdv_id = -1
    for id in vids:
        if id.startswith('v.0.'):
            n = int(id.split('.')[-1])
            max_cav_id = max(max_cav_id, n)
        elif id.startswith('v.1.'):
            m = int(id.split('.')[-1])
            max_hdv_id = max(max_hdv_id, m)

    return max_cav_id, max_hdv_id

def main(demo_mode):
    start_sumo("cfg/vsl.sumo.cfg", False)
    VSL_LANE = ['E1']
    step = 0
    while running(demo_mode, step, 10000):
        if demo_mode and step == 10000:
            print('仿真结束')
            start_sumo("cfg/vsl.sumo.cfg", True)
            step = 0

        traci.simulationStep()

        if step == 0:
            # create_init_vehs()
            # traci.vehicle.add(vehID='v.0.1', routeID="vehicles_route", typeID="passenger",
            #                   departPos=str(25), departLane=str(1), departSpeed=str(15))
            traci.gui.trackVehicle("View #0", 'v.0.1')
            traci.gui.setZoom("View #0", 1500)

        # if step == 200:
        #     ids = traci.vehicle.getIDList()
        #     # print(ids)
        #     print(get_last_id_from_vids(ids))
        #     m, n = get_last_id_from_vids(ids)
        #     create_vehs(m, n)

        if step > 200 and step % 100 == 0:
            vids = traci.vehicle.getIDList()
            for id in vids:
                lane_id = traci.vehicle.getLaneID(id)
                lane = lane_id[:2]
                if lane in VSL_LANE:
                    traci.vehicle.setMaxSpeed(id, speed = 25.00)
                else:
                    traci.vehicle.setMaxSpeed(id, speed = 33.33)
        step += 1

    traci.close()

if __name__ == "__main__":
    main(True)

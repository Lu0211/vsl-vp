#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：programme.py
@File    ：accident.py
@IDE     ：PyCharm 
@Author  ：Lu
@Date    ：2023/4/13 19:56 
'''
import traci


class Accident:
    """
    describe the traffic accident
    :param pos: the position of the traffic accident
    :param lane: the lane of the traffic accident
    :param type: the type of the traffic accident, 1 for accident vehicle, 2 for close the lane
    """
    def __init__(self, pos, lane, type):
        self.pos = pos
        self.lane = lane
        self.type = type
    def setAccident(self):
        if(self.type == 1):
            vehID = "v.2.0"
            traci.vehicle.add(vehID = vehID,routeID="platoon_route",departPos=self.pos,departLane=self.lane)
            traci.vehicle.setLaneChangeMode(vehID = vehID,lcm=0)
            traci.vehicle.setColor(typeID = vehID,color=(255,0,0))
            traci.vehicle.setSpeed(vehID = vehID,speed=0)
            print("Vehicle {} was involved in an accident.".format('c1'))
            return vehID
        elif(self.type == 2):
            lane_id = traci.lane.getIDList()[self.lane - 1]
            traci.lane.setDisallowed(lane_id, ['passenger'])


    def removeAccident(self):
        if(self.type == 1):
            traci.vehicle.remove(vehID='c1')
            print("The accident is removed.")
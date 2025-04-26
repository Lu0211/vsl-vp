#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：scenario_2.py 
@File    ：XML.py
@Author  ：Lu
@Date    ：2023/5/8 16:54 
'''

'''
<e3Detector xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/det_e3_file.xsd">
   <interval begin="0.00" end="10.48" id="ETW1" meanTravelTime="1.59"
             meanOverlapTravelTime="1.72" meanSpeed="31.48" meanHaltsPerVehicle="0.00"
             meanTimeLoss="0.23" vehicleSum="3" meanSpeedWithin="-1.00" meanHaltsPerVehicleWithin="-1.00"
             meanDurationWithin="-1.00" vehicleSumWithin="0" meanIntervalSpeedWithin="-1.00"
             meanIntervalHaltsPerVehicleWithin="-1.00" meanIntervalDurationWithin="-1.00" meanTimeLossWithin="-1.00"/>
</e3Detector>
'''

from xml.dom.minidom import parse

# def GetXML(file):
#     domTree = parse(file)
#
#     rootNode = domTree.documentElement
#     print(rootNode.nodeName)
#     # 获取所有的旅途interval
#     intervals = rootNode.getElementsByTagName("interval")
#     for interval in intervals:
#         print(interval.getAttribute("meanSpeed"))
#
# file = 'new cfg/output_E3.xml'
# GetXML(file)

import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element
# 获取仿真时间内CAV燃油消耗总量
file = 'new cfg/emission.xml'
tree = ET.parse(file)
root = tree.getroot()
cav_total_fuel = 0
for i in root.iter('timestep'):
    for v in i.iter('vehicle'):
        id = v.get('id')
        if id[2] == '0': # CAV
            fuel_value = v.get('fuel')
            cav_total_fuel += float(fuel_value)
            # print('Fuel:',fuel_value)

print(cav_total_fuel)
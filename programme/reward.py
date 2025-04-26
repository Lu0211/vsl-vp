#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：DQN.py 
@File    ：reward.py
@IDE     ：PyCharm 
@Author  ：Lu
@Date    ：2023/4/27 16:11 
'''

from xml.dom.minidom import parse

def GetXML(file):
    domTree = parse(file)

    rootNode = domTree.documentElement
    print(rootNode.nodeName)
    meanSpeed = 0
    meanTravelTime = 0
    # 获取所有的旅途interval
    intervals = rootNode.getElementsByTagName("interval")
    for interval in intervals:
        meanSpeed = interval.getAttribute("meanSpeed")
        meanTravelTime = interval.getAttribute("meanTravelTime")

    return meanSpeed, meanTravelTime

file = 'new cfg/output_E3.xml'
meanSpeed, meanTravelTime = GetXML(file)
print(meanSpeed, meanTravelTime)

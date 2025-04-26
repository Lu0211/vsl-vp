#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：Freeway.py 
@File    ：freeway.py
@Author  ：Lu
@Date    ：2024/6/12 16:09 
'''




import queue
import random
from collections import deque
import traci
import tensorflow as tf
import tensorlayer as tl
import sys
sys.path.append('H:\Lu_programme\programme')
import numpy as np
import plexe
from plexe import Plexe
from plexe import ACC, CACC, FAKED_CACC
from datetime import  datetime
from ccparams import *
from utils import *


simulation_step = 360000
start_sumo("cfg/freeway.sumo.cfg", False, gui=True)
step = 0

while running(True, step, simulation_step):
    if step == simulation_step:
        # start_sumo("cfg/freeway.sumo.cfg", True)
        # step = 0
        print('END')
        break

    traci.simulationStep()

    if step == 10:
        random_vid = traci.vehicle.getIDList()[0]
        traci.gui.trackVehicle("View #0", random_vid)
        traci.gui.setZoom("View #0", 1200)

    # if step % 100 == 0:
    #     vids = traci.lane.getLastStepVehicleIDs('E1_0')
    #     print('vids: ', vids)


    step += 1

traci.close()
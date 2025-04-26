"""
@Author: Fhz
@Create Date: 2023/9/6 17:18
@File: sumoTest.py
@Description: 
@Modify Person Date: 
"""
import sys
import os
import time

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

import traci
from sumolib import checkBinary


if_show_gui = True

if not if_show_gui:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')

sumocfgfile = "sumo_config/my_config_file.sumocfg"
traci.start([sumoBinary, "-c", sumocfgfile])

SIM_STEPS = [1, 100]
beginTime = SIM_STEPS[0]
duration = SIM_STEPS[1]

time.sleep(2)

egoID = "self_car"

for step in range(duration):
    traci.simulationStep(step)
    traci.vehicle.changeLane("self_car", "{}".format(0), 5)


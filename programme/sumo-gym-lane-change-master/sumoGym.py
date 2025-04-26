"""
@Author: Fhz
@Create Date: 2023/9/6 19:52
@File: sumoGym.py
@Description: 
@Modify Person Date: 
"""
import os
import sys

# check if SUMO_HOME exists in environment variable
# if not, then need to declare the variable before proceeding
# makes it OS-agnostic
if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import gym
from gym import spaces
import numpy as np
import traci
from sumolib import checkBinary
from stable_baselines3 import PPO
import time
import argparse


class SumoGym(gym.Env):
    def __init__(self, args):
        self.current_state = None
        self.is_success = False
        self.show_gui = args.show_gui
        self.sumocfgfile = args.sumocfgfile
        self.egoID = args.egoID
        self.frontID = args.frontID
        self.start_time = args.start_time
        self.collision = args.collision
        self.sleep = args.sleep
        self.num_action = args.num_action
        self.lane_change_time = args.lane_change_time

        # Road config
        self.min_x_position = args.min_x_position
        self.max_x_position = args.max_x_position
        self.min_y_position = args.min_y_position
        self.max_y_position = args.max_y_position
        self.min_speed = args.min_speed
        self.max_speed = args.max_speed

        # Reward config
        self.w_time = args.w_time
        self.w_lane = args.w_lane
        self.w_speed = args.w_speed
        self.R_time = args.R_time
        self.P_lane = args.P_lane
        self.V_desired = args.V_desired
        self.R_collision = args.R_collision

        # Done config
        self.target_lane_id = args.target_lane_id
        self.max_count = args.max_count

        self.low = np.array([
            # ego vehicle
            self.min_x_position,
            self.min_y_position,
            self.min_speed,

            # ego leader
            self.min_x_position,
            self.min_y_position,
            self.min_speed,

        ], dtype=np.float32)

        self.high = np.array([
            # ego vehicle
            self.max_x_position,
            self.max_y_position,
            self.max_speed,

            # ego_leader
            self.max_x_position,
            self.max_y_position,
            self.max_speed,

        ], dtype=np.float32)

        self.action_space = spaces.Discrete(self.num_action)
        self.observation_space = spaces.Box(self.low, self.high, dtype=np.float32)

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (
            action,
            type(action),
        )
        if self.is_success:
            self.is_success = False

        if self.collision:
            self.collision = False

        self.count = self.count + 1
        traci.simulationStep(self.count)

        egoLane = traci.vehicle.getLaneIndex(self.egoID)

        if action == 0:
            traci.vehicle.changeLane(self.egoID, "{}".format(0), self.lane_change_time) # lane keeping
        else:
            traci.vehicle.changeLane(self.egoID, "{}".format(1), self.lane_change_time) # lane change

        self.current_state = self.getCurrentStates()

        Collision_Nums = traci.simulation.getCollidingVehiclesNumber()

        if Collision_Nums:
            print("collision num:{}".format(Collision_Nums))
            self.collision = True

        done = self.getDoneState(egoLane)
        reward = self.getRewards()
        info = {}

        if done:
            traci.close()

        return self.current_state, reward, done, info


    def render(self):
        self.show_gui = True
        pass


    def reset(self):

        self.collision = False
        if self.show_gui:
            sumoBinary = checkBinary('sumo-gui')
        else:
            sumoBinary = checkBinary('sumo')

        traci.start([sumoBinary, "-c", self.sumocfgfile])

        if self.sleep:
            time.sleep(2)

        for step in range(self.start_time):
            self.count = step + 1
            traci.simulationStep(self.count)
            if self.count >= 5:
                traci.vehicle.changeLane(self.egoID, "{}".format(0), self.lane_change_time)

        return self.getCurrentStates()


    def getVehicleStateViaId(self, vehicle_ID):
        """
        vehicle_ID: vehicle ID
        function: Get the Vehicle's state via vehicle ID
        """
        curr_pos = traci.vehicle.getPosition(vehicle_ID)
        y_ego, x_ego = curr_pos[0], curr_pos[1]
        speed = traci.vehicle.getSpeed(vehicle_ID)

        return x_ego, y_ego, speed


    def getCurrentStates(self):
        """
        function: Get all the states of vehicles, observation space.
        """

        x_ego, y_ego, speed_ego = self.getVehicleStateViaId(self.egoID)
        x_front, y_front, speed_front = self.getVehicleStateViaId(self.frontID)

        self.current_state = [x_ego, y_ego, speed_ego, x_front, y_front, speed_front]

        return self.current_state


    def getRewards(self):
        """
        function: get the reward after action.
        """

        # Efficient reward
        R_time = - self.R_time
        R_lane = -abs(self.current_state[0] - self.P_lane)
        R_speed = -abs(self.current_state[2] - self.V_desired)

        R_eff = self.w_time * R_time + self.w_lane * R_lane + self.w_speed * R_speed

        # Safety Reward
        if self.collision:
            R_safe = self.R_collision
        else:
            R_safe = 0

        R_eff = max(-30, R_eff)
        R_safe = max(-500, R_safe)

        Rewards = R_eff + R_safe

        return Rewards

    def getDoneState(self, ego_lane):
        """
        function: get the done state of simulation.
        """
        done = False

        if ego_lane == self.target_lane_id and self.current_state[1] >= self.current_state[4] + 30:
            done = True
            print("Mission success!")
            self.is_success = True
            return done

        if self.count >= self.max_count:
            done = True
            print("Exceeding maximum time!")
            return done

        if self.collision:
            done = True
            print("Collision occurs!")
            return done

        return done


def get_args():
    parser = argparse.ArgumentParser()
    # SUMO config
    parser.add_argument("--show_gui", type=bool, default=False, help="The flag of show SUMO gui.")
    parser.add_argument("--sumocfgfile", type=str, default="sumo_config/my_config_file.sumocfg", help="The path of the SUMO configure file.")
    parser.add_argument("--egoID", type=str, default="self_car", help="The ID of ego vehicle.")
    parser.add_argument("--frontID", type=str, default="vehicle1", help="The ID of front vehicle.")
    parser.add_argument("--start_time", type=int, default=10, help="The simulation step before learning.")
    parser.add_argument("--num_action", type=int, default=2, help="The number of action space.")
    parser.add_argument("--lane_change_time", type=float, default=5, help="The time of lane change.")

    # Road config
    parser.add_argument("--min_x_position", type=float, default=0.0, help="The minimum lateral position of vehicle.")
    parser.add_argument("--max_x_position", type=float, default=6.4, help="The maximum lateral position of vehicle.")
    parser.add_argument("--min_y_position", type=float, default=0.0, help="The minimum longitudinal position of vehicle.")
    parser.add_argument("--max_y_position", type=float, default=2000.0, help="The maximum longitudinal position of vehicle.")
    parser.add_argument("--min_speed", type=float, default=0.0, help="The minimum longitudinal speed of vehicle.")
    parser.add_argument("--max_speed", type=float, default=25.0, help="The maximum longitudinal speed of vehicle.")
    parser.add_argument("--count", type=int, default=0, help="The length of a training episode.")
    parser.add_argument("--collision", type=bool, default=False, help="The flag of collision of ego vehicle.")
    parser.add_argument("--sleep", type=bool, default=True, help="The flag of sleep of each simulation.")

    # Reward config
    parser.add_argument("--w_time", type=float, default=0.1, help="The weight of time consuming reward.")
    parser.add_argument("--w_lane", type=float, default=2, help="The weight of target lane reward.")
    parser.add_argument("--w_speed", type=float, default=0.1, help="The weight of desired speed reward.")
    parser.add_argument("--R_time", type=float, default=-1, help="The reward of time consuming.")
    parser.add_argument("--P_lane", type=float, default=-1.6, help="The lateral position of target lane.")
    parser.add_argument("--V_desired", type=float, default=20.0, help="The desired speed.")
    parser.add_argument("--R_collision", type=float, default=-400, help="The reward of ego vehicle collision.")

    # Done config
    parser.add_argument("--target_lane_id", type=int, default=1, help="The ID of target lane.")
    parser.add_argument("--max_count", type=int, default=25, help="The maximum length of a training episode.")

    args = parser.parse_args()

    return args


if __name__ == '__main__':
    args = get_args()
    env = SumoGym(args)

    model = PPO('MlpPolicy', env,
                policy_kwargs=dict(net_arch=[64, 64]),
                learning_rate=5e-4,
                batch_size=32,
                gamma=0.99,
                verbose=1,
                tensorboard_log="logs/",
                )

    model.learn(int(4e4))
    model.save("models/ppo1")


    # ACTIONS_ALL = {
    #     0: 'Lane keeping',
    #     1: 'Lane change',
    # }
    #
    # # load model
    # # model = PPO.load("model_ppo1", env=env)
    #
    # eposides = 10
    #
    # for eq in range(eposides):
    #     print("Test eposide: {}".format(eq))
    #     obs = env.reset()
    #     done = False
    #     rewards = 0
    #     counts = 0
    #     while not done:
    #         counts += 1
    #         time.sleep(0.01)
    #         action = env.action_space.sample()
    #         # action, _state = model.predict(obs, deterministic=True)
    #         # action = action.item()
    #         # print("The action is: {}".format(ACTIONS_ALL[action]))
    #         # print("The action is {}".format(action))
    #         obs, reward, done, info = env.step(action)
    #         # env.render()
    #         rewards += reward
    #     print("The rewards is: {},  the counts is: {}".format(rewards, counts))

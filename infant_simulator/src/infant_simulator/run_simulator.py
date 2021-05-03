#!/usr/bin/env python

'''
Behavior Tree Reward Return 
(Skeleton code for future simulation-generated reward)
Emily Scheide
Oregon State University
March 2020
'''

import rospy
import rospkg
import yaml
from std_msgs.msg import String
from robot import Robot, Controller
from world import World
# from sensor_model import SensorModel
# import random
# import copy
import numpy as np
from parameters import Parameters as p
# import statistics as stats

from behavior_tree.behavior_tree import BehaviorTree

class InfantSimulator:
    def __init__(self):
        self.num_objects = p.n_objects
        self.world_x = p.x_dim
        self.world_y = p.y_dim
        path = '/home/ahelmi/hri_testing/src/belief_bt_generation/behavior_tree/config/crowd_based.tree'
        self.bt = self.get_behavior_tree(path)
        config = 1

        self.world = World(3, 3.5, np.pi)
        # create instance of the world with infant start location and theta
        self.robot = Robot(config, self.bt, self.world, 2, 3, np.pi)
        self.world.robot_create(self.robot)
        self.controller = Controller(self.robot,self.world.inf, self.world)

    def run_sim(self):
        reward, av_distance = self.controller.run()

        return reward, av_distance

    def get_behavior_tree(self, path_to_tree_file):
        return BehaviorTree(path_to_tree_file)


    def generateReward(self, word, max_iterations):

        try:
            # Create BT object from terminal BT CFG
            bt_root, bt = word.createBT()

            print("run_simulator")
            word.printWord()
            #print("len(bt.nodes)", len(bt.nodes))

            robot = Robot(self.config, self.robot_id, self.num_robots, self.seed, bt, max_iterations, self.world)
            # cProfile.run('RobotController(config, robot)')
            robot_controller = RobotController(self.config, robot)
            score, target_reported, belief_distance = robot_controller.run()
            #print('Score: ', score)

            # Get the Word of all active parts of the BT
            active_word = robot.bt_interface.generateActiveCFGWord()
            print("active_word", active_word)
            active_subtree_indices = robot.bt_interface.getActiveSubtreeIndices()
            print('active_subtree_indices', active_subtree_indices)

            #test = [score,target_reported,belief_distance,active_word,active_subtree_indices]
            print('generateReward output', score,target_reported,belief_distance,active_word,active_subtree_indices)
            #print('number of outputs from generateReward', len(test))
            return score, target_reported, belief_distance, active_word, active_subtree_indices

        except rospy.ROSInterruptException: pass

def test():
    sim = InfantSimulator()
    # print('BT node list: ', sim.bt.nodes)
    av_r, av_d = sim.run_sim()
    # score, target_reported, belief_distance, active_word, active_subtree_indices = sim.generateReward(word, sim_iterations)
    return av_r, av_d


if __name__ == "__main__":
    total_r = 0
    total_d = 0
    print('starting simulation')
    rospy.init_node('infant_simulator')
    for i in range(1):
        av_r, av_d = test()
        total_r += av_r
        total_d += av_d
    total_r = np.divide(total_r, 10) # divide by ten trials for average reward
    total_d = np.divide(total_d, 9000) # divide by 10 trials and then 900 iterations to get average distance in a trial
    print(total_r)
    print(total_d)

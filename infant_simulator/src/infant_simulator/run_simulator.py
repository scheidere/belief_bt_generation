#!/usr/bin/env python

'''
Emily Scheide
Oregon State University
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

#from behavior_tree.behavior_tree import BehaviorTree
from behavior_tree.belief_behavior_tree import BeliefBehaviorTree

class InfantSimulator:
    def __init__(self, bt = None):
        self.num_objects = p.n_objects
        self.world_x = p.x_dim
        self.world_y = p.y_dim
        if bt == None:
            path = '/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/move_away.tree'
            self.bt = self.get_behavior_tree(path)
        else:
            self.bt = bt
        config = 1

        # prolly not needed
        # self.collision_penalty = p.coll_penalty
        # self.fail_reward = p.fail_reward
        # self.move_reward = p.move_reward
        # self.collision_count = 0
        # self.reward = 0
        self.world = World(3, 3.5, np.pi)
        # create instance of the world with infant start location and theta
        self.robot = Robot(config, self.bt, self.world, 2, 3, np.pi)
        self.world.robot_create(self.robot)
        self.controller = Controller(self.robot,self.world.inf, self.world)

    def run_sim(self, table_yaml, step, current_score, current_distance, show_plot = True, doing_fancy_plot = True, actual_iteration = None):
        return self.controller.run(table_yaml, step_size = step, starting_score = current_score, starting_distance = current_distance, show_plot=show_plot, doing_fancy_plot = doing_fancy_plot, actual_iteration = actual_iteration)

    def get_behavior_tree(self, path_to_tree_file):
        #return BehaviorTree(path_to_tree_file)
        return BeliefBehaviorTree(path_to_tree_file)

    def update_bt(self, bt):
        self.bt = bt

        # Updates robot.bt and robot.bt_interface
        self.robot.update_bt(bt)


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
    print('BT node list: ', sim.bt.nodes)
    sim.run_sim() # THIS WILL BREAK, NEED UPDATED ARGS
    # score, target_reported, belief_distance, active_word, active_subtree_indices = sim.generateReward(word, sim_iterations)


if __name__ == "__main__":
    print('starting simulation')
    rospy.init_node('infant_simulator')
    test()

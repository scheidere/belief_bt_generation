#!/usr/bin/env python

import rospy
import rospkg
import sys
import yaml
import matplotlib.pyplot as plt
# from world import World
# from world import distance
from scorer import Scorer
from parameters import Parameters as p
# import copy
# import planners
import random
from bt_interface import *
from behavior_tree.behavior_tree import *

import numpy as np
import time


class State:
    def __init__(self):

        # initialized values for conditions (infant distances)
        self.euclidean_diff = 0  # euclidean distance between robot and infant.
        self.euclidean_diff_cat = 2  # Range the infant is away from robot (3 ranges). infant starts nearby the robot
        self.inf_close = 0.3048  # 1 ft
        self.one_ft = self.inf_close
        self.inf_near = 0.9144  # 3 ft
        self.three_ft = self.inf_near
        self.diff_old2new = 0

    def infant2robot_dist(self, world_space):
        """
        Get LIDAR distance to child and categorize if the infant is close, near, or far to the agent
        :return: distance of child from robot, distance category
        """
        # infant distance normalized
        self.euclidean_diff = self.euc_dist(world_space.robot.robot_pos, world_space.infant_pos)
        if self.euclidean_diff <= self.one_ft:
            # close, less than 1 ft
            self.euclidean_diff_cat = 1
            return self.euclidean_diff
        elif self.three_ft >= self.euclidean_diff > self.one_ft:
            # near, less than 3 ft
            self.euclidean_diff_cat = 2
            return self.euclidean_diff
        else:
            # far
            self.euclidean_diff_cat = 3
            return self.euclidean_diff

    def euc_dist(self, robot_pos, infant_pos):
        return np.sqrt((robot_pos[0] - infant_pos[0])**2 + (robot_pos[1] - infant_pos[1])**2)

    # The following are the conditions observed by the robot from the world
    def direct_social_interaction(self, world_space):
        """
        checks if infant is close to robot (<1 ft)
        :return: true if infant is less than 1 ft
        """
        #print('checking dsi')

        self.euclidean_diff = self.infant2robot_dist(world_space)
        if self.euclidean_diff <= self.one_ft:
            # close, less than 1 ft
            self.euclidean_diff_cat = 1
            return True
        else:
            return False

    def social_interaction(self, world_space):
        """
        checks if infant is near robot (1-3 ft)
        :return: true if infant is 1-3ft
        """

        #print('checking si')

        self.euclidean_diff = self.infant2robot_dist(world_space)
        if self.three_ft >= self.euclidean_diff > self.one_ft:
            # near, 1ft - 3ft
            self.euclidean_diff_cat = 2
            return True
        else:
            return False

    def solitary_play(self, world_space):
        """
         checks if infant is far from robot (>3 ft)
         :return: true if infant is >3ft
        """

        #print('checking sp')

        self.euclidean_diff = self.infant2robot_dist(world_space)
        if self.euclidean_diff > self.three_ft:
            # far, more than 3 ft
            self.euclidean_diff_cat = 3
            return True
        else:
            return False

    def relative_child_distance(self, robot, world_space):
        """
        Calculates diff between current infant-robot distance and last
        :return: float value: >0 is approaching, =0 stationary, <0 leaving
        """
        # return True if child moving toward
        new_robot_pos = robot.robot_pos
        new_infant_pos = world_space.infant_pos
        old_infant_pos = world_space.old_infant_pos
        #print('all positions: ', new_robot_pos, new_infant_pos, old_infant_pos)

        if not old_infant_pos:  # if it is None
            #print('there was no previous position')
            world_space.old_infant_pos = new_infant_pos
            return False  # Assume at start child does not move toward

        else:

            distance_oldinf_newrob = self.euc_dist(new_robot_pos, old_infant_pos)
            distance_newinf_newrob = self.euc_dist(new_robot_pos, new_infant_pos)
            self.diff_old2new = distance_oldinf_newrob - distance_newinf_newrob

            #print("Old distance: ", distance_oldinf_newrob, 'New distance: ', distance_newinf_newrob)
            world_space.old_infant_pos = new_infant_pos

            return self.diff_old2new  # positive is child moving towards robot

    def child_moving_toward(self, robot, world_space):
        """
        checks if infant is approaching robot
        :return: true is approaching
        """

        # print('Checking child_moving_toward')

        if self.diff_old2new > 0:
            return True
        return False

    def child_moving_away(self, robot, world_space):
        """
        checks if infant is moving away from robot
        :return: true is moving away
        """

        if self.diff_old2new < 0:
            return True
        return False

    def child_stationary(self, robot, world_space):
        """
        checks if infant is stationary
        :return: true is stationary
        """
        if self.diff_old2new == 0:
            return True
        return False

    def occluded(self, world):
        """
        checks if infant's field of view of the robot is blocked by an obstacle
        :return: true for view is blocked
        """
        # has not been implemented yet due to integration with bt
        pass


class Robot:

    def __init__(self, config, bt, world, x, y, theta):
        self.state = State()
        self.bt = bt
        self.config = config
        self.known_world = world
        self.robot_pos = np.zeros(3)  # x, y, theta
        self.robot_start_pos = np.zeros(3)  # x, y, theta
        self.body_radius = p.agent_rad
        self.sensor_radius = p.detection_radius
        self.buffer = p.buff
        self.d_vel = p.max_vel
        self.set_robot_start_pos(x, y, theta)

        #Set up BT interface
        self.bt_interface = BT_Interface(self.bt)

    def set_robot_start_pos(self, x, y, theta):
        """
        Gives the agent a new starting position in the world
        :return:
        """
        self.robot_start_pos = [x, y, theta]
        self.robot_pos = self.robot_start_pos

    def reset_robot_to_start(self):
        """
        Resets agent to its initial position in the world
        :return:
        """
        self.robot_pos = self.robot_start_pos

    def do_random(self):
        number = np.random.randint(0, 7)
        if number == 0:
            active_actions = ['move_toward']
            action = self.move_toward(self.known_world)
        elif number == 1:
            active_actions = ['move_away']
            action = self.move_away(self.known_world)
        elif number == 2:
            active_actions = ['spin']
            action = self.spin(self.known_world)
        elif number == 3:
            active_actions = ['bubbles']
            action = self.bubbles(self.known_world)
        elif number == 4:
            active_actions = ['lights']
            action = self.lights(self.known_world)
        elif number == 5:
            active_actions = ['sounds']
            action = self.sounds(self.known_world)
        elif number == 6:
            active_actions = ['idle']
            action = self.idle(self.known_world)
        return active_actions

    def do_iteration(self):

        #print('Checking BT node statuses...')
        # for node in self.bt.nodes:
        #     print(node.label, node.status.status)

        #print("++++++++++++++++++++++")
        # print('Checking BT node statuses from bt interface...')
        # for node in self.bt_interface.bt.nodes:
        #     print(node.label, node.status.status)

        # print("++++++++++++++++++++++")

        self.bt_interface.tick_bt()
        # print('Checking BT node statuses PART TWO...')
        # for node in self.bt.nodes:
        #     print(node.label, node.status.status)

        self.condition_updates()  # Conditions: Success or Failure
        # print("++++++++++++++++++++++")

        active_actions = self.bt_interface.getActiveActions()
        #print("+++++++++++++++++++++++++active_actions", active_actions)

        if 'move_toward' in active_actions:

            # print("ACTION EXECUTION: Moving toward!")
            self.move_toward(self.known_world)

        if 'move_away' in active_actions:

            # print("ACTION EXECUTION: Moving away! Aaaaahhh!")
            self.move_away(self.known_world)

        #...


        #if 'move_away' in active_actions: # FINISH THIS FOR CALLS BELOW


        #self.move_away(self.known_world)
        self.spin(self.known_world)
        self.idle(self.known_world)
        self.bubbles(self.known_world)
        self.lights(self.known_world)
        self.sounds(self.known_world)

        
        self.set_action_status() #??? # Actions: Success, Failure, or Running

        #self.move_away(self.known_world)
        #action = self.spin()

        # ??? DOES THIS NEED TO BE IN A LOOP. No.

        return active_actions

    def condition_updates(self):
        """
        Updates all conditions for the bt to observe
        :return:
        """

        # ??? do for all conditions in state class

        # Set condition statuses so they can be updated each iteration
        # and used to choose actions accordingly

        is_in_dsi = self.state.direct_social_interaction(self.known_world)
        is_in_si = self.state.social_interaction(self.known_world)
        is_in_sp = self.state.solitary_play(self.known_world)
        # relative_child_distance must be called to update if child is moving toward/away/stationary
        self.state.relative_child_distance(self, self.known_world)
        is_moving_toward = self.state.child_moving_toward(self, self.known_world)
        is_moving_away = self.state.child_moving_away(self, self.known_world)
        is_stationary = self.state.child_stationary(self, self.known_world)

        # print("Is in dsi? ",is_in_dsi)
        # print("Is in si? ",is_in_si)
        # print("Is in sp? ",is_in_sp)
        # print("Is child moving toward? ", is_moving_toward)
        # print("Is child moving away? ", is_moving_away)
        # print("Is child stationary? ", is_stationary)

        # Ensure string is exact condition name
        self.bt_interface.setConditionStatus('direct_social_interaction', is_in_dsi)
        self.bt_interface.setConditionStatus('social_interaction', is_in_si)
        self.bt_interface.setConditionStatus('solitary_play', is_in_sp)
        self.bt_interface.setConditionStatus('child_moving_toward', is_moving_toward)
        self.bt_interface.setConditionStatus('child_moving_away', is_moving_away)
        self.bt_interface.setConditionStatus('child_stationary', is_stationary)
        # self.bt_interface.setConditionStatus('occluded', is_occluded)
        # ... more conditions

    def get_quotient(self, delta_y, delta_x):

        try:
            return delta_y/delta_x
        except ZeroDivisionError:
            return np.inf

    def move_toward(self, world_space):
        """
        move towards the infant based on the time step
        :return: 
        """

        #print("Checking if move_toward active")

        #active_actions = self.bt_interface.getActiveActions()

        # print("current pos: ", self.robot_pos)

        world_space.change_robot_plot_color()


        # 0,0 is at bottom left of world map
        # Baseline orientation 0 is to the right

        delta_x = world_space.infant_pos[0] - self.robot_pos[0] 
        delta_y = world_space.infant_pos[1] - self.robot_pos[1]

        #xy_quotient = self.get_quotient(delta_y, delta_x)

        # if delta_x == 0:
        #     delta_x = 0.0001
        # if delta_y == 0:
        #     delta_y = 0.0001

        theta_new = np.arctan2(delta_y,delta_x)

        time_step = 1
        # # move towards the infant
        # if (self.robot_pos[0] == world_space.infant_pos[0]) or (self.robot_pos[1] == world_space.infant_pos[1]):
        #     # lie along one of the parallels
        #     if self.robot_pos[0] == world_space.infant_pos[0]:
        #         # lie along x, check which y is bigger for direction
        #         if self.robot_pos[1] > world_space.infant_pos[1]:
        #             theta_new = np.pi/2
        #         else:
        #             theta_new = 3*np.pi/2
        #     else:
        #         if self.robot_pos[0] > world_space.infant_pos[0]:
        #             theta_new = np.pi
        #         else:
        #             theta_new = 0
        # if (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
        #     # top right quadrant movement
        #     theta_new = np.random.uniform(0, np.pi/2)
        # elif (self.robot_pos[0] > world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
        #     # top left quadrant movement
        #     theta_new = np.random.uniform(np.pi/2,np.pi)
        # elif (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] > world_space.infant_pos[1]):
        #     # bottom right quadrant movement
        #     theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
        # else:
        #     # bottom left quadrant movement
        #     theta_new = np.random.uniform(np.pi, 3*np.pi/2)

        # move it!
        x_new = self.robot_pos[0] + self.d_vel * time_step * np.cos(theta_new)
        y_new = self.robot_pos[1] + self.d_vel * time_step * np.sin(theta_new)

        # Boundary checking -- if the new positions are illegal, reject and set collision = true
        # Keep the previous coordinates
        illegal = self.collision_detection(world_space.objects, x_new, y_new, world_space.world_x, world_space.world_y)
        if not illegal:
            self.robot_pos = [x_new, y_new, theta_new]
            # print('new pos: ', self.robot_pos)
            return True

        return False # leaving for when you uncomment illegal stuff


    def move_away(self, world_space):
        """
        move away form the infant based on the time step
        :return:
        """

        # print("current pos: ", self.robot_pos)

        world_space.change_robot_plot_color()

        delta_x = world_space.infant_pos[0] - self.robot_pos[0] 
        delta_y = world_space.infant_pos[1] - self.robot_pos[1]

        theta_new = np.arctan2(delta_y,delta_x) + np.pi


        time_step = 1
        # if (self.robot_pos[0] == world_space.infant_pos[0]) or (self.robot_pos[1] == world_space.infant_pos[1]):
        #     # lie along one of the parallels
        #     if self.robot_pos[0] == world_space.infant_pos[0]:
        #         # lie along x, check which y is bigger for direction
        #         if self.robot_pos[1] > world_space.infant_pos[1]:
        #             theta_new = 3*np.pi/2
        #         else:
        #             theta_new = np.pi/2
        #     else:
        #         if self.robot_pos[0] > world_space.infant_pos[0]:
        #             theta_new = 0
        #         else:
        #             theta_new = np.pi
        # if (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
        #     # bottom left quadrant movement
        #     theta_new = np.random.uniform(np.pi, 3*np.pi/2)

        # elif (self.robot_pos[0] > world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
        #     # bottom right quadrant movement
        #     theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
        # elif (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] > world_space.infant_pos[1]):
        #     # top left quadrant movement
        #     theta_new = np.random.uniform(np.pi/2,np.pi)
        # else:
        #     # top right quadrant movement
        #     theta_new = np.random.uniform(0, np.pi/2)

        # move it!
        x_new = self.robot_pos[0] + self.d_vel * time_step * np.cos(theta_new)
        y_new = self.robot_pos[1] + self.d_vel * time_step * np.sin(theta_new)

        # Boundary checking -- if the new positions are illegal, reject and set collision = true
        # Keep the previous coordinates
        illegal = self.collision_detection(world_space.objects, x_new, y_new, world_space.world_x, world_space.world_y)
        if not illegal:
            self.robot_pos = [x_new, y_new, theta_new]
            # print('new pos: ', self.robot_pos)
            return True

        return False


    def bubbles(self, world_space):
        """
        function for the robot to blow bubbles.
        :return:
        """
        # give it the world so we can check if we are above the counter

        active_actions = self.bt_interface.getActiveActions()

        if 'bubbles' in active_actions:

            # print("ACTION EXECUTION: Blowing bubbles! Ultimate party time!")

            action = world_space.bubbles()
            world_space.change_robot_plot_color('bubbles')
            return action

        return None

    def idle(self, world_space):
        """
        function for the robot to stand still
        :return:
        """

        active_actions = self.bt_interface.getActiveActions()

        if 'idle' in active_actions:

            # print("ACTION EXECUTION: Idle, doing nothing! Not party time!")
            world_space.change_robot_plot_color()
            # robot location does not change, just return true this action happened
            return True

        return None

    def spin(self, world_space):
        """
        function for the robot to spin in place
        """

        active_actions = self.bt_interface.getActiveActions()

        if 'spin' in active_actions:

            # print("ACTION EXECUTION: Spinning around! Time to party!")

            # spin to a random orientation
            theta_new = np.random.uniform(0, 2 * np.pi)
            self.robot_pos = [self.robot_pos[0], self.robot_pos[1], theta_new]
            world_space.change_robot_plot_color()
            return True

        return None

    def lights(self, world_space):
        """
        function for the robot to flash Lights
        :return:
        """
        # maybe we change the icon color or something of the robot to show it did this?

        active_actions = self.bt_interface.getActiveActions()

        if 'lights' in active_actions:

            # print("ACTION EXECUTION: Flashing lights! Time to party!")
            world_space.change_robot_plot_color('lights')
            return True

        return None

    def sounds(self, world_space):
        """
        function for the robot to play sounds
        :return:
        """
        # same as lights, change somehow to show it happened?

        active_actions = self.bt_interface.getActiveActions()

        if 'sounds' in active_actions:

            # print("ACTION EXECUTION: Making sound! Time to party!")
            world_space.change_robot_plot_color('sounds')
            return True

        return None

    def collision_detection(self, wld_obj, x_new, y_new, world_x, world_y):
        """
        This function is called every time step to detect if the agent has run into anything
        Calculates in C-space
        :return: True for collision, false for no collision
        """

        collision = False
        buff = self.body_radius + self.buffer  # Acceptable distance to wall

        if x_new <= 0 + buff or x_new >= world_x - buff:  # Checks outer wall
            collision = True
        elif y_new <= 0 + buff or y_new >= world_y - buff:  # Checks outer wall
            collision = True
        
        # check for collisions with objects
        for i, obj in enumerate(wld_obj):
            if (x_new + buff > obj[0][0] and x_new - buff < obj[2][0]) \
                and (y_new + buff > obj[0][1] and y_new - buff < obj[2][1]):
                collision = True

        return collision

    def set_action_status(self):
        """
        Pulls the active actions from the bt interface and updates their status
        :return:
        """
        active_actions = self.bt_interface.getActiveActions()
        # does this mean we are doing an action?


        for action in active_actions:
            if action == 'move_toward':
                # does this need a failure node, same with move toward and away?
                self.bt_interface.setActionStatusSuccess(action)
            elif action == 'move_away':
                # does this need a failure node, same with move toward and away?
                self.bt_interface.setActionStatusSuccess(action)
            elif action == 'spin':
                self.bt_interface.setActionStatusSuccess(action)
            elif action == 'idle':
                self.bt_interface.setActionStatusSuccess(action)
            elif action == 'bubbles':
                # does this need a failure node, same with move toward and away?
                self.bt_interface.setActionStatusSuccess(action)
            elif action == 'lights':
                self.bt_interface.setActionStatusSuccess(action)
            elif action == 'sounds':
                self.bt_interface.setActionStatusSuccess(action)


            # elif action == 'go_to_child': # not using for first testing round
            #     if self.planner_type == Robot.PLANNER_TYPE_SHORTEST:
            #         self.bt_interface.setActionStatusRunning(action)
            #     else:
            #         self.bt_interface.setActionStatusSuccess(action)
            
            else:
                pass
                # print('action',action)
                # print("set_action_status: Action does not exist")


class Controller:
    def __init__(self, robot, infant, world):
        self.robot = robot
        self.infant = infant
        self.world = world

    def run(self):
        """
        Main function where simulation iterations occur. The robot and infant tick
        based on this function
        :return:
        """
        # start_time = rospy.Time.now()
        score = Scorer()
        num_iterations = 0

        r = rospy.Rate(1) # 1hz
        # self.world.infant_pos_update()
        # self.world.robot_pos_update()
        # self.world.world_plot(num_iterations)
        # num_iterations += 1
        # r.sleep()

        while not rospy.is_shutdown() and num_iterations < 300:
            #for i in range(10): # test loop, need to use above ros method
            #print(' ')
            #print("iteration: " + str(num_iterations))


            # active_actions = self.robot.do_random()
            active_actions = self.robot.do_iteration()
            # print("Active: ", active_actions, " Iterations: ", num_iterations)
            #print("Active ids: ", self.robot.bt.active_ids)


            infant_action = self.infant.infant_step(self.robot.robot_pos, active_actions, self.world.centers, self.world.objects)
            score.infant_sim_reward(infant_action, self.robot.state.infant2robot_dist(self.world))

            self.world.infant_pos_update()
            self.world.robot_pos_update()
            self.world.world_plot(num_iterations)
            # print('Infant action: ', infant_action)
            num_iterations += 1
            print("iterations: ", num_iterations)
            #time.sleep(1)
            #r.sleep()
            
        return score.score, score.distance  




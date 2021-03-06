#!/usr/bin/env python

import rospy
import rospkg
import sys
import yaml
import matplotlib.pyplot as plt
# from world import World
# from world import distance
# from scorer import Scorer
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
        print('checking dsi')

        self.euclidean_diff = self.infant2robot_dist(world_space)
        if self.euclidean_diff <= self.inf_close:
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

        print('checking si')

        self.euclidean_diff = self.infant2robot_dist(world_space)
        if self.euclidean_diff <= self.inf_near:
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

        print('checking sp')

        self.euclidean_diff = self.infant2robot_dist(world_space)
        if self.euclidean_diff > self.inf_near:
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

        print('Checking child_moving_toward')

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
        self.bt_interface = BT_Interface(bt)

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

    def do_iteration(self):

        self.bt_interface.tick_bt()

        active_actions = self.bt_interface.getActiveActions()
        #print("+++++++++++++++++++++++++active_actions", active_actions)

        self.condition_updates()  # Conditions: Success or Failure
        
        self.set_action_status() #??? # Actions: Success, Failure, or Running

        action = self.move_toward(self.known_world)
        # action = self.move_away(self.known_world)
        #action = self.spin()

        # ??? DOES THIS NEED TO BE IN A LOOP

        return action, active_actions

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

        #print("Is in dsi? ",is_in_dsi)
        #print("Is in si? ",is_in_si)
        #print("Is in sp? ",is_in_sp)
        print("Is child moving toward? ", is_moving_toward)
        print("Is child moving away? ", is_moving_away)
        print("Is child stationary? ", is_stationary)

        # Ensure string is exact condition name
        self.bt_interface.setConditionStatus('direct_social_interaction', is_in_dsi)
        self.bt_interface.setConditionStatus('social_interaction', is_in_si)
        self.bt_interface.setConditionStatus('solitary_play', is_in_sp)
        self.bt_interface.setConditionStatus('child_moving_toward', is_moving_toward)
        self.bt_interface.setConditionStatus('child_moving_away', is_moving_away)
        # self.bt_interface.setConditionStatus('occluded', is_occluded)
        # ... more conditions

    def move_toward(self, world_space):
        """
        move towards the infant based on the time step
        :return: 
        """

        print("Checking if move_toward active")

        active_actions = self.bt_interface.getActiveActions()

        if 'move_toward' in active_actions:

            print("ACTION EXECUTION: Moving toward!")

            ### NEEDS TIME STEP ####

            time_step = 1
            # move towards the infant
            if (self.robot_pos[0] == world_space.infant_pos[0]) or (self.robot_pos[1] == world_space.infant_pos[1]):
                # lie along one of the parallels
                if self.robot_pos[0] == world_space.infant_pos[0]:
                    # lie along x, check which y is bigger for direction
                    if self.robot_pos[1] > world_space.infant_pos[1]:
                        theta_new = np.pi/2
                    else:
                        theta_new = 3*np.pi/2
                else:
                    if self.robot_pos[0] > world_space.infant_pos[0]:
                        theta_new = np.pi
                    else:
                        theta_new = 0
            if (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
                # top right quadrant movement
                theta_new = np.random.uniform(0, np.pi/2)
            elif (self.robot_pos[0] > world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
                # top left quadrant movement
                theta_new = np.random.uniform(np.pi/2,np.pi)
            elif (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] > world_space.infant_pos[1]):
                # bottom right quadrant movement
                theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
            else:
                # bottom left quadrant movement
                theta_new = np.random.uniform(np.pi, 3*np.pi/2)

            # move it!
            x_new = self.robot_pos[0] + self.d_vel * time_step * np.cos(theta_new)
            y_new = self.robot_pos[1] + self.d_vel * time_step * np.sin(theta_new)

            # Boundary checking -- if the new positions are illegal, reject and set collision = true
            # Keep the previous coordinates
            illegal = self.collision_detection(world_space.objects, x_new, y_new, world_space.world_x, world_space.world_y)
            if not illegal:
                self.robot_pos = [x_new, y_new, theta_new]
                return True

            return False

        return None

    def move_away(self, world_space):
        """
        move away form the infant based on the time step
        :return:
        """

        active_actions = self.bt_interface.getActiveActions()

        if 'move_away' in active_actions:

            print("ACTION EXECUTION: Moving away! Aaaaahhh!")

            ### NEEDS TIME STEP ####

            time_step = 1
            if (self.robot_pos[0] == world_space.infant_pos[0]) or (self.robot_pos[1] == world_space.infant_pos[1]):
                # lie along one of the parallels
                if self.robot_pos[0] == world_space.infant_pos[0]:
                    # lie along x, check which y is bigger for direction
                    if self.robot_pos[1] > world_space.infant_pos[1]:
                        theta_new = 3*np.pi/2
                    else:
                        theta_new = np.pi/2
                else:
                    if self.robot_pos[0] > world_space.infant_pos[0]:
                        theta_new = 0
                    else:
                        theta_new = np.pi
            if (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
                # bottom left quadrant movement
                theta_new = np.random.uniform(np.pi, 3*np.pi/2)

            elif (self.robot_pos[0] > world_space.infant_pos[0]) and (self.robot_pos[1] < world_space.infant_pos[1]):
                # bottom right quadrant movement
                theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
            elif (self.robot_pos[0] < world_space.infant_pos[0]) and (self.robot_pos[1] > world_space.infant_pos[1]):
                # top left quadrant movement
                theta_new = np.random.uniform(np.pi/2,np.pi)
            else:
                # top right quadrant movement
                theta_new = np.random.uniform(0, np.pi/2)

            # move it!
            x_new = self.robot_pos[0] + self.d_vel * time_step * np.cos(theta_new)
            y_new = self.robot_pos[1] + self.d_vel * time_step * np.sin(theta_new)

            # Boundary checking -- if the new positions are illegal, reject and set collision = true
            # Keep the previous coordinates
            illegal = self.collision_detection(world_space.objects, x_new, y_new, world_space.world_x, world_space.world_y)
            if not illegal:
                self.robot_pos = [x_new, y_new, theta_new]
                return True

            return False
        return None

    def bubbles(self, world_space):
        """
        function for the robot to blow bubbles.
        :return:
        """
        # give it the world so we can check if we are above the counter

        active_actions = self.bt_interface.getActiveActions()

        if 'bubbles' in active_actions:

            print("ACTION EXECUTION: Blowing bubbles! Ultimate party time!")

            action = world_space.bubbles()
            return action

        return None

    def idle(self):
        """
        function for the robot to stand still
        :return:
        """

        active_actions = self.bt_interface.getActiveActions()

        if 'idle' in active_actions:

            print("ACTION EXECUTION: Idle, doing nothing! Not party time!")

            # robot location does not change, just return true this action happened
            return True

        return None

    def spin(self):
        """
        function for the robot to spin in place
        """

        active_actions = self.bt_interface.getActiveActions()

        if 'spin' in active_actions:

            print("ACTION EXECUTION: Spinning around! Time to party!")

            # spin to a random orientation
            theta_new = np.random.uniform(0, 2 * np.pi)
            self.robot_pos = [self.robot_pos[0], self.robot_pos[1], theta_new]
            return True

        return None

    def lights(self):
        """
        function for the robot to flash Lights
        :return:
        """
        # maybe we change the icon color or something of the robot to show it did this?

        active_actions = self.bt_interface.getActiveActions()

        if 'lights' in active_actions:

            print("ACTION EXECUTION: Flashing lights! Time to party!")

            return True

        return None

    def sounds(self):
        """
        function for the robot to play sounds
        :return:
        """
        # same as lights, change somehow to show it happened?

        active_actions = self.bt_interface.getActiveActions()

        if 'sounds' in active_actions:

            print("ACTION EXECUTION: Making sound! Time to party!")

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
        
        # if current_action != active_actions(whichever is active) and timer has exceeded timer (2 seconds for example):
        #     current_action = active_actions (whichever is active/running?)
        #     end timer for action
        #     calculate relative distance change based on timer (send it start and end timer)
        #     start new_timer for action


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
                print('action',action)
                print("set_action_status: Action does not exist")


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
        num_iterations = 0

        rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            #for i in range(10): # test loop, need to use above ros method
            print(' ')
            print("iteration: " + str(num_iterations))
            self.world.infant_pos_update()
            self.world.robot_pos_update()
            self.world.world_plot()
            robot_action, active_actions = self.robot.do_iteration()

            print("Active ids: ", self.robot.bt.active_ids)

            num_iterations += 1
            
            if robot_action == True:
                robot_action = 1
            
            # print("Robot location is: ", self.robot.robot_pos)

            # INCOMPLETE: Infant reaction to robot action
            # if active_actions == Running???
            # we return active actions above, meaning all active actions (just one prob) infant can react to 
            # infant iteration?
            # we call infant_step only 20% of the time and infant does action
            # if random number > 10:
                # we decided to do something different
            infant_action = self.infant.infant_step(self.robot.robot_pos, robot_action, self.world.centers)

            # print('Infant action: ', infant_action)
            time.sleep(3)
            

            
        # INCOMPLETE: will need to create scorer and return score
        return 1  # self.robot.basestation_scorer.score, self.robot.has_reported, self.robot.basestation_scorer.belief_distance



# # Main function.
# if __name__ == '__main__':
    
#     # Initialise the node
#     rospy.init_node('robot')
#     # Get the config file etc
#     rospack = rospkg.RosPack()
#     filepath = rospack.get_path('simulator') + "/config/" + rospy.get_param('~config')
#     with open(filepath, 'r') as stream:
#         config = yaml.safe_load(stream)
#     robot_id = rospy.get_param('~robot_id')
#     num_robots = rospy.get_param('~num_robots')
#     seed = rospy.get_param('~seed')

#     try:
#         # cfg_word = createWord('?  (  ->  (  (in_comms)  ?  (  (battery_low)  [random_walk]  )  )  ->  (  ?  (  [go_to_comms]  [report]  )  [report]  ?  (  (at_surface)  (benign_object_found)  [take_to_drop_off]  )  )  )')
#         # bt_root, bt = cfg_word.createBT()

#         cfg_word = createWord('?  (  ->  (  (is_armed)  [disarm]  )  ->  (  ?  (  [shortest_path]  [shortest_path]  [shortest_path]  )  ?  (  [shortest_path]  [shortest_path]  [shortest_path]  [shortest_path]  )  )  ) ')
#         bt_root, bt = cfg_word.createBT()

        
#         max_iterations = 1000

#         # Create the world
#         world = World(config)
#         do_test = True # don't error check graph

#         world.init_world(seed, do_test)

#         robot = Robot(config, robot_id, num_robots, seed, bt, max_iterations, world)
#         # cProfile.run('RobotController(config, robot)')
#         robot_controller = RobotController(config, robot)
#         score, target_reported, distance = robot_controller.run()
#         print('Score: ', score)
#         if target_reported:
#             print('Has reported:', target_reported, 'not necessarily correctly')
#         else:
#             print('Has reported:', target_reported)
#     except rospy.ROSInterruptException: pass
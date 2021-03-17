#!/usr/bin/env python3

import numpy as np
from math import sin, cos
from parameters import Parameters as p


class Infant:

    def __init__(self, x, y, theta):
        self.infant_pos = np.zeros(3)  # x, y, theta
        self.infant_pos_old  = np.zeros(3)
        self.infant_start_pos = np.zeros(3)  # x, y, theta
        self.inf_table = np.zeros((6,9))
        self.buffer = p.buff
        #self.inf_close = 0.3048 # 1 ft
        #self.inf_near = 0.9144 # 3 ft
        self.one_ft = 0.3048 # 1 ft
        self.three_ft = 0.9144 # 3 ft
        self.body_radius = p.infant_rad
        self.dist_cat = 2 # distance category, starts at close
        self.dist = 0 # distance of infant to robot
        self.inf_vel = p.inf_vel
        self.inf_grav = p.inf_gravity
        self.obj_check = np.zeros(p.n_objects) # true/false array for occlusion
        self.inf_action = 0
        self.init_inf_probability_table()
        self.set_infant_start_pos(x, y, theta)

    def set_infant_start_pos(self, x, y, theta):
        """
        Gives the agent a new starting position in the world (Complete world reset)
        :return:
        """
        self.infant_start_pos = [x, y, theta]
        self.infant_pos = self.infant_start_pos

    def reset_infant_to_start(self):
        """
        Resets agent to its initial position in the world
        :return:
        """
        self.infant_pos = self.infant_start_pos

    def infant2robot_dist(self, robot_pos):
        """
        Get LIDAR distance to child and categorize if the infant is close, near, or far to the agent
        :return: distance of child from robot, distance category
        """
        # infant distance normalized
        self.euclidean_diff = self.euc_dist(robot_pos)
        if self.euclidean_diff <= self.one_ft:
            # close, less than 1 ft
            self.euclidean_diff_cat = 0
        elif self.three_ft >= self.euclidean_diff > self.one_ft:
            # near, less than 3 ft
            self.euclidean_diff_cat = 1
        else:
            # far
            self.euclidean_diff_cat = 2

        return self.euclidean_diff_cat

    def euc_dist(self, robot_pos):
        return np.sqrt((robot_pos[0] - self.infant_pos[0])**2 + (robot_pos[1] - self.infant_pos[1])**2) 

    def inf_obj_dist(self, wld_cent):
        '''
        determine which object the child is closest to so they can move towards it
        '''
        obj_val = 0
        dist = 0
        prev_dist = 0

        for i, obj in enumerate(wld_cent):
            dist = np.sqrt((self.infant_pos[0] - obj[0])**2 + (self.infant_pos[1] - obj[1])**2)
            if dist < prev_dist:
                prev_dist = dist
                obj_val = i
        return obj_val

    def init_inf_probability_table(self):
        """
        fill infant probability table for determining infant actions
        """
        # Each row has prob that infant is moving toward for each robot action
        # Each row has that info for a different distance (dsi, si, sp)
        self.inf_table = np.array([[4,7,4,6,8,7,5], [4,7,4,6,8,7,6], [5,6,4,5,9,7,6]])
        self.inf_table = np.divide(self.inf_table, float(10)) 


    def infant_table_decrease(self, agent_action):
        """
        reduce the probability of the robot action causing movement in a linear fashion as the actions will get repetitive
        """
        if self.inf_table[agent_action[0]][agent_action[1]] <= 0.1:
            self.inf_table[agent_action[0]][agent_action[1]] = 0.1
        else:
            self.inf_table[agent_action[0]][agent_action[1]] -= 0.05        


    def infant_occlusion(self, robot_pos, wld_obj, wld_x, wld_y):
        """
        Determine if infant is visible to robot or occluded by object. Infant is visible if distance is close but may not be visible
        We assume height is not relevant yet for occlusion
        :return: true/false if infant is occluded to agent or visible
        TRUE MEANS INFANT CANNOT SEE ROBOT
        """
        # first check if child is even facing robot, then check if object is in the way
        if (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
            # x and y less than robot, bottom left of robot
            if self.infant_pos[2] >= (3*np.pi)/2:
                return False
        elif (self.infant_pos[0] > robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
            # x greater, y less than so infant bottom right of robot
            if self.infant_pos[2] <= np.pi/2:
                return False
        elif (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] > robot_pos[1]):
            # x less, y greater so infant top left of robot
            if self.infant_pos[2] >= np.pi and self.infant_pos[2] <= (3*np.pi)/2:
                return False
        else:
            # x and y greater so infant top right of robot
            if self.infant_pos[2] >= np.pi/2 and self.infant_pos[2] <= np.pi:
                return False

        self.obj_check = [True, True, True]
        for i,obj in enumerate(wld_obj):
            # first check if both are above the bottom part of the object
            if (self.infant_pos[1] > obj[0][1]) and (robot_pos[1] > obj[0][1]):
                # both are above the bottom part of the object, check the top part
                if (self.infant_pos[1] > obj[2][1]) and (robot_pos[1] > obj[2][1]):
                    # they can see each other
                    self.obj_check[i] = False
                # at least robot or infant is below top part of the object
                else:
                    # is the infant to the left of the object wall?
                    if self.infant_pos[0] < obj[0][0]:
                        # is the robot also to the left of the object wall?
                        if robot_pos[0] < obj[0][0]:
                            # they can see each other
                            self.obj_check[i] = False
                        else:
                            # can't see each other
                            self.obj_check[i] = True
                    else:
                        # infant is to the right of the wall, is the robot also?
                        if robot_pos[0] > obj[0][0]:
                            self.obj_check[i] = False
                        else:
                            self.obj_check[i] = True
            # both below bottom part of object
            elif (self.infant_pos[1] < obj[0][1]) and (robot_pos[1] < obj[0][1]):
                self.obj_check[i] = False
            # one is above or below the bottom part of the object, check left right
            # both to right of left side of object
            elif (self.infant_pos[0] > obj[0][0]) and (robot_pos[0] > obj[0][0]):
                # both are to the right of far side of object
                if (self.infant_pos[0] > obj[1][0]) and (robot_pos[0] > obj[1][0]):
                    self.obj_check[i] = False
                else:
                    self.obj_check[i] = True
            # both to left of object
            elif (self.infant_pos[0] < obj[0][0]) and (robot_pos[0] < obj[0][0]):
                self.obj_check[i] = False
        # if any object occludes, return True
        if np.any(self.obj_check):
            return True
        else:
            return False
            

    def collision_detection(self, x_new, y_new, world_x, world_y):
        """
        This function is called every time step to detect if the child has run into anything
        Calculates in C-space
        :return: True for collision, false for no collision
        """

        collision = False
        buff = self.body_radius + self.buffer  # Acceptable distance to wall

        if x_new <= 0 + buff or x_new >= world_x - buff:  # Checks outer wall
            collision = True
        elif y_new <= 0 + buff or y_new >= world_y - buff:  # Checks outer wall
            collision = True
        
        return collision

    def infant_step(self, robot_pos, agent_action, wld_centers):
        """
        infant takes an action
        1 is infant moves towards the robot
        2 is infant moves away from the robot
        3 is infant movement towards an object
        4 is stationary
        :return: True if infant moves towards robot, False is infant does anything else
        """
        time_step = 1
        # get random number for checking if action happens
        check_val = np.random.random()
        dist_cat = self.infant2robot_dist(robot_pos) 
        # e.g.: if robot action = bubbles, infant does action with X% probability
        try:
            robot_action = p.agent_actions[active_actions[-1]]
        except:
            robot_action = p.agent_actions['idle'] 
        action_prob = self.inf_table[dist_cat, robot_action]
        # print("probability of response by infant: ", action_prob) 

        # IF YOU WANT CHILD TO ALWAYS MOVE TOWARD (for testing): Set check_val to lesser value than action_prob
        # check_val = 1
        # action_prob = 2

        if check_val <= action_prob:
            # success, infant moves towards robot
            # print("Infant is now moving toward the robot")
            self.inf_action = 1
            if (self.infant_pos[0] == robot_pos[0]) or (self.infant_pos[1] == robot_pos[1]):
                # lie along one of the parallels
                if self.infant_pos[0] == robot_pos[0]:
                    # lie along x, check which y is bigger for direction
                    if self.infant_pos[1] > robot_pos[1]:
                        theta_new = np.pi/2
                    else:
                        theta_new = 3*np.pi/2
                else:
                    if self.infant_pos[0] > robot_pos[0]:
                        theta_new = np.pi
                    else:
                        theta_new = 0
            if (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
                # top right quadrant movement
                theta_new = np.random.uniform(0, np.pi/2)
            elif (self.infant_pos[0] > robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
                # top left quadrant movement
                theta_new = np.random.uniform(np.pi/2, np.pi)
            elif (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] > robot_pos[1]):
                # bottom right quadrant movement
                theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
            else:
                # bottom left quadrant movement
                theta_new = np.random.uniform(np.pi, 3*np.pi/2)
            # move it!
            x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
            y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
            self.infant_pos_old = self.infant_pos
            illegal = self.collision_detection(x_new, y_new, p.x_dim, p.y_dim) # Emily addition to prevent leaving map
            if not illegal: # Emily addition to prevent leaving map
                self.infant_pos = [x_new, y_new, theta_new]
                # infant moved towards the robot so return True
                return True
            self.infant_pos = [x_new, y_new, theta_new]
            return True

        # elif check_val >= self.inf_grav:   
        #         # gravity movement towards an object
        #         self.inf_action = 3
        #         # find the closest object and let the infant move towards it
        #         closest_obj = self.inf_obj_dist(wld_centers)
        #         if (self.infant_pos[0] == wld_centers[closest_obj][0]) or (self.infant_pos[1] == wld_centers[closest_obj][1]):
        #         # lie along one of the parallels
        #             if self.infant_pos[0] == wld_centers[closest_obj][0]:
        #                 # lie along x, check which y is bigger for direction
        #                 if self.infant_pos[1] > wld_centers[closest_obj][1]:
        #                     theta_new = np.pi/2
        #                 else:
        #                     theta_new = 3*np.pi/2
        #             else:
        #                 if self.infant_pos[0] > wld_centers[closest_obj][0]:
        #                     theta_new = np.pi
        #                 else:
        #                     theta_new = 0
        #         if (self.infant_pos[0] < wld_centers[closest_obj][0]) and (self.infant_pos[1] < wld_centers[closest_obj][1]):
        #             # top right quadrant movement
        #             theta_new = np.random.uniform(0, np.pi/2)
        #         elif (self.infant_pos[0] > wld_centers[closest_obj][0]) and (self.infant_pos[1] < wld_centers[closest_obj][1]):
        #             # top left quadrant movement
        #             theta_new = np.random.uniform(np.pi/2,np.pi)
        #         elif (self.infant_pos[0] < wld_centers[closest_obj][0]) and (self.infant_pos[1] > wld_centers[closest_obj][1]):
        #             # bottom right quadrant movement
        #             theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
        #         else:
        #             # bottom left quadrant movement
        #             theta_new = np.random.uniform(np.pi, 3*np.pi/2)
        #         # move it!
        #         x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
        #         y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
        #         self.infant_pos_old = self.infant_pos
        #         illegal = self.collision_detection(x_new, y_new, p.x_dim, p.y_dim) # Emily addition to prevent leaving map
        #         if not illegal: # Emily addition to prevent leaving map
        #             self.infant_pos = [x_new, y_new, theta_new]
        #             return False
        #         self.infant_pos = [x_new, y_new, theta_new]

        # failure, child either moves away or stays still
        elif check_val >= 0.5:
            # move away
            self.inf_action = 2
            if (self.infant_pos[0] == robot_pos[0]) or (self.infant_pos[1] == robot_pos[1]):
            # lie along one of the parallels
                if self.infant_pos[0] == robot_pos[0]:
                # lie along x, check which y is bigger for direction
                    if self.infant_pos[1] > robot_pos[1]:
                        theta_new = 3*np.pi/2
                    else:
                        theta_new = np.pi/2
                else:
                    if self.infant_pos[0] > robot_pos[0]:
                        theta_new = np.pi
                    else:
                        theta_new = 0
            if (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
                # bottom left quadrant movement
                theta_new = np.random.uniform(np.pi, (3*np.pi)/2)
            elif (self.infant_pos[0] > robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
                # bottom right quadrant movement
                theta_new = np.random.uniform((3*np.pi)/2, 2* np.pi)
            elif (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] > robot_pos[1]):
                # top left quadrant movement
                theta_new = np.random.uniform(np.pi/2, np.pi)
            else:
                # top right quadrant movement
                theta_new = np.random.uniform(0, np.pi/2)
            # move it!
            x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
            y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
            self.infant_pos_old = self.infant_pos
            illegal = self.collision_detection(x_new, y_new, p.x_dim, p.y_dim) # Emily addition to prevent leaving map
            if not illegal: # Emily addition to prevent leaving map
                self.infant_pos = [x_new, y_new, theta_new]
                return False
            self.infant_pos = [x_new, y_new, theta_new]

        else:
            # stationary
            self.inf_action = 4

        return False
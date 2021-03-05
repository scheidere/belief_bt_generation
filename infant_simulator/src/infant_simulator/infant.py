#!/usr/bin/env python3

import numpy as np
from math import sin, cos


class Infant:

    def __init__(self, p, x, y, theta):
        self.infant_pos = np.zeros(3)  # x, y, theta
        self.infant_pos_old  = np.zeros(3)
        self.infant_start_pos = np.zeros(3)  # x, y, theta
        self.inf_table = np.zeros((6,9))
        self.buffer = p.buff
        self.inf_close = 0.3048 # 1 ft
        self.inf_near = 0.9144 # 3 ft
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
        self.inf_table = np.array([[4,7,4,6,8,7,5,7,8], [2,2,1,2,4,3,5,4,4], [4,7,4,6,8,7,6,7,7],
         [2,2,1,2,4,3,5,4,4], [5,6,4,5,9,7,6,6,6], [2,1,1,2,4,3,5,5,5]])
        self.inf_table = np.divide(self.inf_table, float(10))


    def infant_table_decrease(self, agent_action):
        """
        reduce the probability of the robot action causing movement in a linear fashion as the actions will get repetitive
        """
        if self.inf_table[agent_action[0]][agent_action[1]] <= 0.1:
            self.inf_table[agent_action[0]][agent_action[1]] = 0.1
        else:
            self.inf_table[agent_action[0]][agent_action[1]] -= 0.05        


    def infant_occlusion(self, agent_pos, wld_obj, wld_x, wld_y):
        """
        Determine if infant is visible to robot or occluded by object. Infant is visible if distance is close but may not be visible
        We assume height is not relevant yet for occlusion
        :return: true/false if infant is occluded to agent or visible
        TRUE MEANS INFANT CANNOT SEE ROBOT
        """
        # first check if child is even facing robot, then check if object is in the way
        if (self.infant_pos[0] < agent_pos[0]) and (self.infant_pos[1] < agent_pos[1]):
            # x and y less than robot, bottom left of robot
            if self.infant_pos[2] >= (3*np.pi)/2:
                return False
        elif (self.infant_pos[0] > agent_pos[0]) and (self.infant_pos[1] < agent_pos[1]):
            # x greater, y less than so infant bottom right of robot
            if self.infant_pos[2] <= np.pi/2:
                return False
        elif (self.infant_pos[0] < agent_pos[0]) and (self.infant_pos[1] > agent_pos[1]):
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
            if (self.infant_pos[1] > obj[0][1]) and (agent_pos[1] > obj[0][1]):
                # both are above the bottom part of the object, check the top part
                if (self.infant_pos[1] > obj[2][1]) and (agent_pos[1] > obj[2][1]):
                    # they can see each other
                    self.obj_check[i] = False
                # at least robot or infant is below top part of the object
                else:
                    # is the infant to the left of the object wall?
                    if self.infant_pos[0] < obj[0][0]:
                        # is the robot also to the left of the object wall?
                        if agent_pos[0] < obj[0][0]:
                            # they can see each other
                            self.obj_check[i] = False
                        else:
                            # can't see each other
                            self.obj_check[i] = True
                    else:
                        # infant is to the right of the wall, is the robot also?
                        if agent_pos[0] > obj[0][0]:
                            self.obj_check[i] = False
                        else:
                            self.obj_check[i] = True
            # both below bottom part of object
            elif (self.infant_pos[1] < obj[0][1]) and (agent_pos[1] < obj[0][1]):
                self.obj_check[i] = False
            # one is above or below the bottom part of the object, check left right
            # both to right of left side of object
            elif (self.infant_pos[0] > obj[0][0]) and (agent_pos[0] > obj[0][0]):
                # both are to the right of far side of object
                if (self.infant_pos[0] > obj[1][0]) and (agent_pos[0] > obj[1][0]):
                    self.obj_check[i] = False
                else:
                    self.obj_check[i] = True
            # both to left of object
            elif (self.infant_pos[0] < obj[0][0]) and (agent_pos[0] < obj[0][0]):
                self.obj_check[i] = False
        # if any object occludes, return True
        if np.any(self.obj_check):
            return True
        else:
            return False
            

    def infant_step(self, agent_pos, agent_action, wld_centers):
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
        # e.g.: if robot action = bubbles, infant does action with X% probability
        #action_prob = self.inf_table[agent_action[0]][agent_action[1]]
        action_prob = 0.3
        if check_val <= action_prob:
            # success, infant moves towards robot
            print("Infant is now moving toward the robot")
            self.inf_action = 1
            if (self.infant_pos[0] == agent_pos[0]) or (self.infant_pos[1] == agent_pos[1]):
                # lie along one of the parallels
                if self.infant_pos[0] == agent_pos[0]:
                    # lie along x, check which y is bigger for direction
                    if self.infant_pos[1] > agent_pos[1]:
                        theta_new = np.pi/2
                    else:
                        theta_new = 3*np.pi/2
                else:
                    if self.infant_pos[0] > agent_pos[0]:
                        theta_new = np.pi
                    else:
                        theta_new = 0
            if (self.infant_pos[0] < agent_pos[0]) and (self.infant_pos[1] < agent_pos[1]):
                # top right quadrant movement
                theta_new = np.random.uniform(0, np.pi/2)
            elif (self.infant_pos[0] > agent_pos[0]) and (self.infant_pos[1] < agent_pos[1]):
                # top left quadrant movement
                theta_new = np.random.uniform(np.pi/2, np.pi)
            elif (self.infant_pos[0] < agent_pos[0]) and (self.infant_pos[1] > agent_pos[1]):
                # bottom right quadrant movement
                theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
            else:
                # bottom left quadrant movement
                theta_new = np.random.uniform(np.pi, 3*np.pi/2)
            # move it!
            x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
            y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
            self.infant_pos_old = self.infant_pos
            self.infant_pos = [x_new, y_new, theta_new]
            # infant moved towards the robot so return True
            return True


        elif check_val >= self.inf_grav:   
                # gravity movement towards an object
                self.inf_action = 3
                # find the closest object and let the infant move towards it
                closest_obj = self.inf_obj_dist(wld_centers)
                if (self.infant_pos[0] == wld_centers[closest_obj][0]) or (self.infant_pos[1] == wld_centers[closest_obj][1]):
                # lie along one of the parallels
                    if self.infant_pos[0] == wld_centers[closest_obj][0]:
                        # lie along x, check which y is bigger for direction
                        if self.infant_pos[1] > wld_centers[closest_obj][1]:
                            theta_new = np.pi/2
                        else:
                            theta_new = 3*np.pi/2
                    else:
                        if self.infant_pos[0] > wld_centers[closest_obj][0]:
                            theta_new = np.pi
                        else:
                            theta_new = 0
                if (self.infant_pos[0] < wld_centers[closest_obj][0]) and (self.infant_pos[1] < wld_centers[closest_obj][1]):
                    # top right quadrant movement
                    theta_new = np.random.uniform(0, np.pi/2)
                elif (self.infant_pos[0] > wld_centers[closest_obj][0]) and (self.infant_pos[1] < wld_centers[closest_obj][1]):
                    # top left quadrant movement
                    theta_new = np.random.uniform(np.pi/2,np.pi)
                elif (self.infant_pos[0] < wld_centers[closest_obj][0]) and (self.infant_pos[1] > wld_centers[closest_obj][1]):
                    # bottom right quadrant movement
                    theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
                else:
                    # bottom left quadrant movement
                    theta_new = np.random.uniform(np.pi, 3*np.pi/2)
                # move it!
                x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
                y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
                self.infant_pos_old = self.infant_pos
                self.infant_pos = [x_new, y_new, theta_new]

        # failure, child either moves away or stays still
        elif check_val >= 0.5:
            # move away
            self.inf_action = 2
            if (self.infant_pos[0] == agent_pos[0]) or (self.infant_pos[1] == agent_pos[1]):
            # lie along one of the parallels
                if self.infant_pos[0] == agent_pos[0]:
                # lie along x, check which y is bigger for direction
                    if self.infant_pos[1] > agent_pos[1]:
                        theta_new = 3*np.pi/2
                    else:
                        theta_new = np.pi/2
                else:
                    if self.infant_pos[0] > agent_pos[0]:
                        theta_new = np.pi
                    else:
                        theta_new = 0
            if (self.infant_pos[0] < agent_pos[0]) and (self.infant_pos[1] < agent_pos[1]):
                # bottom left quadrant movement
                theta_new = np.random.uniform(np.pi, (3*np.pi)/2)
            elif (self.infant_pos[0] > agent_pos[0]) and (self.infant_pos[1] < agent_pos[1]):
                # bottom right quadrant movement
                theta_new = np.random.uniform((3*np.pi)/2, 2* np.pi)
            elif (self.infant_pos[0] < agent_pos[0]) and (self.infant_pos[1] > agent_pos[1]):
                # top left quadrant movement
                theta_new = np.random.uniform(np.pi/2, np.pi)
            else:
                # top right quadrant movement
                theta_new = np.random.uniform(0, np.pi/2)
            # move it!
            x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
            y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
            self.infant_pos_old = self.infant_pos
            self.infant_pos = [x_new, y_new, theta_new]


        else:
            # stationary
            self.inf_action = 4

        return False
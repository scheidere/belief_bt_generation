#!/usr/bin/env python3

import numpy as np
from math import sin, cos
from parameters import Parameters as p
import random


class Infant:

    def __init__(self, x, y, theta):
        self.infant_pos = np.zeros(3)  # x, y, theta
        self.infant_pos_old  = np.zeros(3)
        self.infant_start_pos = np.zeros(3)  # x, y, theta
        self.inf_table = np.zeros((6,9)) #probability table
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
        self.robot_counter = 0
        self.toy_counter = 0
        self.still_counter = 0
        self.move_counter = 0
        self.state = 0
        self.robot_interaction_prob = p.uninterest_robot_interaction_prob
        self.toy_interaction_prob = p.uninterest_toy_interaction_prob
        self.robot_playtime = p.uninterest_robot_playtime
        self.toy_playtime = p.uninterest_toy_playtime

    def change_to_interest(self):
        self.robot_interaction_prob = p.interest_robot_interaction_prob
        self.toy_interaction_prob = p.interest_toy_interaction_prob
        self.robot_playtime = p.interest_robot_playtime
        self.toy_playtime = p.interest_toy_playtime
    
    def change_to_uninterested(self):
        self.robot_interaction_prob = p.uninterest_robot_interaction_prob
        self.toy_interaction_prob = p.uninterest_toy_interaction_prob
        self.robot_playtime = p.uninterest_robot_playtime
        self.toy_playtime = p.uninterest_toy_playtime

    def change_to_random(self):
        self.robot_interaction_prob = p.random_robot_interaction_prob
        self.toy_interaction_prob = p.random_toy_interaction_prob
        self.robot_playtime = p.random_robot_playtime
        self.toy_playtime = p.random_toy_playtime


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
        prev_dist = 100

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
    
    def see_robot(self, robot_pos, wld_obj): 
        return self.infant_occlusion(robot_pos,wld_obj)

    def see_toy(self, wld_centers):
        #print("total center", wld_centers)
        for i, cntr in enumerate(wld_centers):
               #print("i", i)
               #print("center", cntr)
               #print("center x", cntr[0])
               #print("center y", cntr[1])
            if self.infant_pos[2] <= np.pi/2:
                if cntr[0] >= 3 and cntr[1] >=3:
                    toy_quadrant = 1
                    #print("toy in quadrant 1")
                    return True
            elif self.infant_pos[2] <= np.pi and self.infant_pos[2] >= np.pi/2:
                if cntr[0] >= 3 and cntr[1] < 3:
                    toy_quadrant = 2
                    #print("toy in quadrant 2")
                    return True
            elif self.infant_pos[2] <= (3* np.pi)/2 and self.infant_pos[2]  >= np.pi:
                if cntr[0] < 3 and cntr[1] < 3:
                    toy_quadrant = 3
                    #print("toy in quadrant 3")
                    return True
            else: 
                if cntr[0] < 3 and cntr[1] >= 3:
                    toy_quadrant = 4
                    #print("toy in quadrant 4")
                    return True 

#state 0: look away 
#state 1: keep looking
#state 2: stay attuned robot
#state 3: stay attuned toy

    def state_machine(self, wld_centers, robot_pos, wld_obj):
        self.robot_counter = 0
        self.toy_counter = 0
        self.still_counter = 0
        self.move_counter = 0
        if not(self.see_robot(robot_pos, wld_obj)) and self.see_toy(wld_centers):
            interact_prob1 = random.randint(1, 1000)
            if interact_prob1 <= self.toy_interaction_prob:
                interact_prob2 = random.randint(1,1000)
                if interact_prob2 <= 965:
                    self.state = 3
                    #print("I am attuned to toy")
                else:
                    self.state = 0
                    #print("I am looking away")
            elif interact_prob1 >= self.toy_interaction_prob and interact_prob1 <= (self.toy_interaction_prob + self.robot_interaction_prob):
                interact_prob2 = random.randint(1, 1000)
                if interact_prob2 <= 965:
                    self.state = 2
                    #print("I am attuned to the robot")
                else:
                    self.state = 1
                    #("I am looking away")
            else:
                interact_prob2 = random.randint(1, 1000)
                if interact_prob2 <= 965:
                    self.state = 0
                    #print("I am looking away")
                else:
                    self.state = 1
                    #print("I will keep looking")

        elif not(self.see_robot(robot_pos, wld_obj)):
            interact_prob1 = random.randint(1, 1000)
            if interact_prob1 <= self.robot_interaction_prob: 
                interact_prob2 = random.randint(1, 1000)
                if interact_prob2 <= 965:
                    self.state = 2
                    #print("I am attuned to the robot")
                else:
                    self.state = 0
                    #("I am looking away")
            else:
                interact_prob2 = random.randint(1, 1000)
                if interact_prob2 <= 965:
                    self.state = 0
                    #print("I am looking away")
                else:
                    self.state = 1
                    #print("I will keep looking")

        elif self.see_toy(wld_centers):
            interact_prob1 = random.randint(1, 1000)
            if interact_prob1 <= self.toy_interaction_prob:
                interact_prob2 = random.randint(1, 1000)
                if interact_prob2 <= 965:
                    self.state = 3
                    #print("I am staying attuned to toy")
                else:
                    self.state = 0
                    #print("I am looking away from toy")
            else:
                interact_prob2 = random.randint(1, 1000)
                if interact_prob2 <= 965:
                    self.state = 0
                    #print("I am looking away")
                else:
                    self.state = 1
                    #print("I will keep looking")
        else:
            interact_prob1 = random.randint(1, 1000)
            if interact_prob1 <= 965:
                self.state = 0
                #print("I am looking away")
            else:
                self.state = 1
                #print("I will keep looking")
        return self.state


    def infant_occlusion(self, robot_pos, wld_obj):
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

    def infant_step(self, robot_pos, active_actions, wld_centers, wld_obj):
        time_step = 1
        self.change_to_interest()
        #self.change_to_uninterested()
        #self.change_to_random()

        
        if self.state == 2:
            self.robot_counter = self.robot_counter + 1
        #     theta_old = self.infant_pos[2]
            if self.robot_counter == self.robot_playtime:
                 self.state_machine(wld_centers, robot_pos, wld_obj)
                
            self.infant_pos_old = self.infant_pos
            
            if round(self.infant_pos[0], 1) == round(robot_pos[0], 1) or round(self.infant_pos[1], 1) == round(robot_pos[1], 1):
                pass
            else:
                delta_x = robot_pos[0] - self.infant_pos[0]
                delta_y = robot_pos[1] - self.infant_pos[1]

                theta_new = np.arctan2(delta_y,delta_x)
                
                x_new = self.infant_pos[0] + self.inf_vel * time_step * np.cos(theta_new)
                y_new = self.infant_pos[1] + self.inf_vel * time_step * np.sin(theta_new)
                self.infant_pos_old = self.infant_pos
                self.infant_pos = [x_new, y_new, theta_new]
                # print("infant pos", self.infant_pos)
                # print("robot pos", robot_pos)
        #     if (self.infant_pos[0] == robot_pos[0]) or (self.infant_pos[1] == robot_pos[1]):
        #      # lie along one of the parallels
        #         if self.infant_pos[0] == robot_pos[0]:
        #             # lie along x, check which y is bigger for direction
        #             if self.infant_pos[1] > robot_pos[1]:
        #                     theta_new = np.pi/2
        #             else:
        #                     theta_new = 3*np.pi/2
        #         else:
        #             if self.infant_pos[0] > robot_pos[0]:
        #                     theta_new = np.pi
        #             else:
        #                     theta_new = 0
        #     if (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
        #             # top right quadrant movement
        #             theta_new = np.random.uniform(0, np.pi/2)

        #             print("quadrant 1")
        #     elif (self.infant_pos[0] > robot_pos[0]) and (self.infant_pos[1] < robot_pos[1]):
        #             # top left quadrant movement
        #             theta_new = np.random.uniform(np.pi/2, np.pi)
        #             print("quadrant 2")
        #     elif (self.infant_pos[0] < robot_pos[0]) and (self.infant_pos[1] > robot_pos[1]):
        #             # bottom right quadrant movement
        #             theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)
        #     else:
        #             # bottom left quadrant movement
        #             theta_new = np.random.uniform(np.pi, 3*np.pi/2)
        #             print("quadrant 3")
        #         # move it!
        #     x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
        #     y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
        #     self.infant_pos_old = self.infant_pos
        #     illegal = self.collision_detection(x_new, y_new, p.x_dim, p.y_dim) # Emily addition to prevent leaving map
        #     if not illegal: # Emily addition to prevent leaving map
        #         self.infant_pos = [x_new, y_new, theta_new]
        #         # infant moved towards the robot so return True
        #         return True
        #     self.infant_pos = [x_new, y_new, theta_new]
        #     return True

        elif self.state == 3:
            # print("following toy")
            self.toy_counter = self.toy_counter + 1
            if self.toy_counter == self.toy_playtime:
                self.state_machine(wld_centers, robot_pos, wld_obj)
            
            closest_obj = self.inf_obj_dist(wld_centers)
            if round(self.infant_pos[0], 1) == round(wld_centers[closest_obj][0], 1) or round(self.infant_pos[1], 1) == round(wld_centers[closest_obj][1], 1):
                pass
            else:
                delta_x = wld_centers[closest_obj][0] - self.infant_pos[0]
                delta_y = wld_centers[closest_obj][1] - self.infant_pos[1]

                theta_new = np.arctan2(delta_y,delta_x)
                
                x_new = self.infant_pos[0] + self.inf_vel * time_step * np.cos(theta_new)
                y_new = self.infant_pos[1] + self.inf_vel * time_step * np.sin(theta_new)
                self.infant_pos_old = self.infant_pos
                self.infant_pos = [x_new, y_new, theta_new]

            # closest_obj = self.inf_obj_dist(wld_centers)
            # theta_old = self.infant_pos[2]
            # if (self.infant_pos[0] == wld_centers[closest_obj][0]) or (self.infant_pos[1] == wld_centers[closest_obj][1]):
            #     # lie along one of the parallels
            #     if self.infant_pos[0] == wld_centers[closest_obj][0]:
            #             # lie along x, check which y is bigger for direction
            #         if self.infant_pos[1] > wld_centers[closest_obj][1]:
            #                 theta_new = np.pi/2
            #         else:
            #                 theta_new = 3*np.pi/2
            #     else:
            #         if self.infant_pos[0] > wld_centers[closest_obj][0]:
            #                 theta_new = np.pi
            #         else:
            #                 theta_new = 0
            # if (self.infant_pos[0] < wld_centers[closest_obj][0]) and (self.infant_pos[1] < wld_centers[closest_obj][1]):
            #         # top right quadrant movement
            #         theta_new = np.random.uniform(0, np.pi/2)

            # elif (self.infant_pos[0] > wld_centers[closest_obj][0]) and (self.infant_pos[1] < wld_centers[closest_obj][1]):
            #         # top left quadrant movement
            #         theta_new = np.random.uniform(np.pi/2,np.pi)
            # elif (self.infant_pos[0] < wld_centers[closest_obj][0]) and (self.infant_pos[1] > wld_centers[closest_obj][1]):
            #         # bottom right quadrant movement
            #         theta_new = np.random.uniform((3*np.pi)/2, 2*np.pi)    
            # else:
            #         # bottom left quadrant movement
            #         theta_new = np.random.uniform(np.pi, 3*np.pi/2)
            #     # move it!
            # x_new = self.infant_pos[0] + self.inf_vel * time_step * cos(theta_new)
            # y_new = self.infant_pos[1] + self.inf_vel * time_step * sin(theta_new)
            # self.infant_pos_old = self.infant_pos
            # illegal = self.collision_detection(x_new, y_new, p.x_dim, p.y_dim) # Emily addition to prevent leaving map
            # if not illegal: # Emily addition to prevent leaving map
            #     self.infant_pos = [x_new, y_new, theta_new]
            #     return False
            # self.infant_pos = [x_new, y_new, theta_new]
            
        elif self.state == 1:
            self.still_counter = self.still_counter + 1
            if self.still_counter == 2:
                self.state_machine(wld_centers, robot_pos, wld_obj)
            # print("staying still")
        elif self.state == 0:
            self.move_counter = self.move_counter + 1
            if self.move_counter == 2:
                self.state_machine(wld_centers, robot_pos, wld_obj)
            # print("moving random!")
            what_degree = random.randint(0, 10)
            new_degree = 10 * np.pi/180
            old_theta = self.infant_pos[2]
            if what_degree <=5:
                new_theta = new_degree + old_theta
            else:
                new_theta = new_degree - old_theta
            new_x = self.infant_pos[0] + self.inf_vel * time_step * cos(new_theta)
            new_y = self.infant_pos[1] + self.inf_vel * time_step * sin(new_theta)
            self.infant_pos_old = self.infant_pos
            self.infant_pos = [new_x, new_y, new_theta]
            return True
        else:
            self.state_machine(wld_centers, robot_pos, wld_obj)
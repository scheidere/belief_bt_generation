#!/usr/bin/env python

# import rospy
# import rospkg
# import sys
# import yaml
import random
import numpy as np
# import copy
# import cPickle as pickle
# from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

from parameters import Parameters as p
from math import sin, cos
from infant import Infant
from robot import Robot, Controller
 

class World():

    def __init__(self, infant_x, infant_y, infant_theta):

        # Create a world
        self.num_objects = p.n_objects
        self.world_x = p.x_dim
        self.world_y = p.y_dim
        self.objects = np.zeros((self.num_objects, 4), dtype=(float,2))
        self.centers = np.zeros((self.num_objects,2))
        
        # UNCOMMENT IF YOU WANT OBSTACLES
        self.gen_world_objects(self.num_objects)

        # set up bubble cap
        self.bubble_cap = p.bubble_cap
        self.bubble_use = 0 # bubble use counter

        # make an infant
        self.inf = Infant(infant_x, infant_y, infant_theta)
        self.infant_pos_update() # sets self.infant_pos
        self.old_infant_pos = None


        # plotting config
        self.h_state = None
        self.infant_plot = 0
        self.robot_plot = 0
        self.infant_plot_arrow = 0
        self.robot_plot_arrow = 0
        self.image_counter = 0
        self.markercolor = None
        self.iteration_marker = None
        # default images of robot and baby
        self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo.png'
        self.infant_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/bb.png'
        self.zoom = 0.04

    def infant_pos_update(self):
        self.infant_pos = self.inf.infant_pos
        # updates infant instance every tick

    def robot_pos_update(self):
        self.robot_pos = self.robot.robot_pos
        # updates robot instance every tick

    def getImage(self, path, zoom):
        return OffsetImage(plt.imread(path, 0), zoom=zoom)

    def robot_create(self, robot):
        self.robot = robot

    def change_robot_plot_color(self, robot_action=None):
        print('robot_action', robot_action)

        # if not these actions, then goes to default robot picture
        if robot_action == None:
            self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo.png'
            self.zoom = 0.04
        elif robot_action == 'bubbles':
            self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo_bubbles.png'
            self.zoom = 0.1
        elif robot_action == 'lights':
            self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo_lights.png'
            self.zoom = 0.1
        elif robot_action == 'sounds':
            self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo_sounds.png'
            self.zoom = 0.1
        elif robot_action == 'all':
            self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo_all.png'
            self.zoom = 0.1
        elif robot_action == 'bubbles_sounds':
            self.robot_path = '/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/robo_bubbles_sounds.png'
            self.zoom = 0.1

    def gen_world_objects(self, num_objects):
        """
        add a couple random sized objects to the environment which may occlude infant to robot
        creates array of objects with x,y coordinates for each corner of the object
        """
        for obj in range(num_objects):
            # random x corner, random y corner, random width, random height
            obj_x = np.random.uniform(0, self.world_x-1)
            obj_y = np.random.uniform(0, self.world_y-1)
            obj_w = np.random.uniform(0.5, 1.21)
            obj_h = np.random.uniform(0.5, 1.21)
            # bottom left corner, bottom right corner, top right corner, top left corner 
            self.objects[obj] = ((obj_x, obj_y), (obj_x + obj_w, obj_y), (obj_x + obj_w, obj_y + obj_h), (obj_x, obj_y + obj_h))
            self.centers[obj] = (obj_x + obj_w / 2, obj_y + obj_h / 2)

    def world_plot(self, iteration):
        """
        Whole plotter function that includes initializing, updating, and saving the plot.
        :return:
        """
        # plt.ion()
        # fig = plt.figure(1)
        fig , ax = plt.subplots(1)
        ax.set_xlim([0,self.world_x])
        ax.set_ylim([0,self.world_y])
        ax.set_yticks([])
        ax.set_xticks([])

        # for i in range(self.num_objects):
        #         for k in range(3):
        #             plt.plot((self.objects[i][k][0],self.objects[i][k+1][0]),(self.objects[i][k][1],self.objects[i][k+1][1]),'r')
        #         plt.plot((self.objects[i][3][0],self.objects[i][0][0]),(self.objects[i][3][1],self.objects[i][0][1]),'r')


        ab = AnnotationBbox(self.getImage(self.robot_path, zoom=self.zoom), [self.robot_pos[0],self.robot_pos[1]], frameon=False)
        ax.add_artist(ab)
        ab = AnnotationBbox(self.getImage(self.infant_path, zoom=0.04), [self.infant_pos[0],self.infant_pos[1]], frameon=False)
        ax.add_artist(ab)

        endy = 0.2 * np.sin(self.infant_pos[2])
        endx = 0.2 * np.cos(self.infant_pos[2])
        roby = 0.2 * np.sin(self.robot_pos[2])
        robx = 0.2 * np.cos(self.robot_pos[2])
        fig.canvas.draw()

        # plt.show()
        # if self.h_state == None:
            # self.image_counter = 0
            # for i in range(self.num_objects):
            #     for k in range(3):
            #         plt.plot((self.objects[i][k][0],self.objects[i][k+1][0]),(self.objects[i][k][1],self.objects[i][k+1][1]),'r')
            #     plt.plot((self.objects[i][3][0],self.objects[i][0][0]),(self.objects[i][3][1],self.objects[i][0][1]),'r')

            # self.infant_plot = plt.plot(self.infant_pos[0],self.infant_pos[1], marker=".", markersize=10)
            # self.robot_plot = plt.plot(self.robot_pos[0],self.robot_pos[1], marker=".", markersize=10) #, markerfacecolor='b')
            # self.iteration_marker = plt.text(4.5, 5.5, str(iteration), fontsize=10)
            # # get direction of infant and agent
            # self.infant_plot_arrow = plt.plot([self.infant_pos[0], self.infant_pos[0]+endx], [self.infant_pos[1], self.infant_pos[1]+endy])
            # self.robot_plot_arrow = plt.plot([self.robot_pos[0], robx+self.robot_pos[0]], [self.robot_pos[1], roby+self.robot_pos[1]])
            # fig.canvas.draw()
            # self.h_state = True
            
        # else:
        #     # self.image_counter += 1
        #     self.infant_plot[0].set_xdata(self.infant_pos[0])
        #     self.infant_plot[0].set_ydata(self.infant_pos[1])
        #     self.robot_plot[0].set_xdata(self.robot_pos[0])
        #     self.robot_plot[0].set_ydata(self.robot_pos[1])
        #     self.robot_plot[0].set_markerfacecolor(self.markercolor)
        #     self.robot_plot_arrow[0].set_data([self.robot_pos[0], self.robot_pos[0]+robx], [self.robot_pos[1], self.robot_pos[1]+roby])
        #     self.infant_plot_arrow[0].set_data([self.infant_pos[0], self.infant_pos[0]+endx], [self.infant_pos[1], self.infant_pos[1]+endy])
        #     self.iteration_marker.set_text(str(iteration))
        #     fig.canvas.draw()
        #     plt.pause(0.1)
        ##plt.savefig("/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/infant_simulator/src/infant_simulator/images/test_" + str(self.image_counter) + ".png")  # , bbox='tight')  # , bbox_extra_artists=[legend])
        plt.savefig("/home/ahelmi/hri_testing/src/belief_bt_generation/infant_simulator/src/infant_simulator/images/test_" + str(iteration) + ".png")  # , bbox='tight')  # , bbox_extra_artists=[legend])
        # # self.iteration_marker

    def bubbles(self):
        """
        iterate bubble counter or return false if we are over
        """
        # check if above bubble counter
        if self.bubble_use > self.bubble_cap:
            return False
        else:
            self.bubble_use += 1

        return True



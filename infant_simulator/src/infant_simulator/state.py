#!/usr/bin/env python

import numpy as np

class WorldState:
    def __init__(self, table_yaml, DEBUG = False):

        # Inputs will be pulled from yaml elsewhere, 
        # right before wherever an instance of this class is initialized
        # Inputs should be lists

        self.table_yaml = table_yaml
        self.condition_table_list = self.table_yaml['condition_table']

        self.DEBUG = DEBUG

        self.initStateDict()

        print("INITIALIZING STATE")


    def initStateDict(self):

        self.state = {}

        for i in range(len(self.condition_table_list)):

            condition_info_dictionary = self.condition_table_list[i]
            if self.DEBUG:
                print('cond dict', condition_info_dictionary)
            condition = condition_info_dictionary['condition']
            self.state[condition] = 'R' #default because unknown?


    def updateState(self, condition_string_list, status_list):

        for i in range(len(condition_string_list)):
            if self.DEBUG:
                print(self.state[condition_string_list[i]])
                print(status_list[i])
            self.state[condition_string_list[i]] = status_list[i]


    # def update_state(self, bt):

    #     ??? this needs to check all conditions not just those in tree

    #     for node in bt.nodes:
    #         print(node.label, node.status.status)

    #         if node.label in self.state.keys():
    #             if node.status.status == 0: # failure
    #                 status = 'F'
    #             elif node.status.status == 1:
    #                 status = 'R'
    #             elif node.status.status == 2:
    #                 status = 'S'
    #             else:
    #                 print('____________NODE STATUS ERROR____________')

    #             self.state[node.label] = status

    #         #if isinstance(node, Condition):
#!/usr/bin/env python

import numpy as np


class State:
    def __init__(self, table_yaml, DEBUG = False):

        # Inputs will be pulled from yaml elsewhere, 
        # right before wherever an instance of this class is initialized
        # Inputs should be lists

        self.table_yaml = table_yaml
        self.condition_table_list = self.table_yaml['condition_table']

        self.DEBUG = DEBUG

        self.initStateDict()
        

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

    def __eq__(self, other_state):

        # Need to access state variable in state object 
        # Can't just compare instances 

        # First confirm the other_state input is an instance of state class
        if not isinstance(other_state, State):
            # Don't compared against unrelated types
            return NotImplemented

        return self.state == other_state.state



#??? make state generation take a bt, pull nodes, differentiate between those of same type
#and retain associated returnstatus (either Success or failure)
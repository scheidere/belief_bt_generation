#!/usr/bin/env python

import yaml

from world_simulator.state import State
from belief_state import BeliefState, combine


def general_state_test(table_yaml):

    print("##### Running general_state_test #####")

    # Testing state class (see state.py in world_simulator)
    #conditions_string_list = ['condition 1','condition 2']
    #conditions_string_list = ['social_interaction', 'occluded']
    #statuses = ['S','F'] #[2, 0] # success, failure?
    
    init_state = State(table_yaml)

    if DEBUG:
        print('state', init_state.state)

    return init_state

def yaml_test():

    print("##### Running yaml_test #####")

    # Read yaml file
    with open('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/belief_behavior_tree/src/input/action_tables.yaml', 'r') as stream:
        try:
            #print(yaml.safe_load(stream))
            action_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    if DEBUG:
        # Get the table, in the form of a list
        action_table_list = action_yaml['action_table']
        print(action_table_list)

        # Get a "row," i.e. info for a particular action
        action_info_dictionary = action_table_list[1]
        print(action_info_dictionary)
        print(type(action_info_dictionary))

        # Get action name
        action_name = action_info_dictionary['action']
        print(action_name)

        # Get postconditions
        postconditions_list = action_info_dictionary['postconditions']
        print('Postconditions: ')
        print(postconditions_list)

        # Get postconditions for prob 1
        prob1_postconditions_dict = postconditions_list[0]
        print(prob1_postconditions_dict)

        # Get prob 1
        prob_1 = prob1_postconditions_dict.keys()[0]
        print(prob_1)

        # Get postcondition list from prob 1 dictionary
        prob1_postconditions_list = prob1_postconditions_dict[prob_1]
        print(prob1_postconditions_list)

        # Print single postcondition
        postcondition = prob1_postconditions_list[0]
        print(postcondition)

        postcondition_string = postcondition[0]
        print(postcondition_string)

        postcondition_status = postcondition[1]
        print(postcondition_status) # e.g. 'F'

    return action_yaml

def get_table_yaml():
    # Read yaml file
    with open('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/belief_behavior_tree/src/input/action_tables.yaml', 'r') as stream:
        try:
            #print(yaml.safe_load(stream))
            table_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    return table_yaml


def general_belief_state_test(init_state, table_yaml):

    print("##### Running general_belief_state_test #####")

    # Testing belief state class
    states = [init_state]
    probabilities = [1]
    init_belief_state = BeliefState(states, probabilities, table_yaml)
    if DEBUG:
        print('belief_state', init_belief_state.belief)

    return init_belief_state

def combine_belief_states_test(table_yaml):

    print("##### Running combine_belief_states_test #####")

    # Combine function test (see combine in belief_state.py)
    mem_1 = BeliefState([],[], table_yaml)
    mem_1.belief = [(0.3, 'state_dict', 'S'), (0.1, 'state_dict2', 'S'), (0.1, 'state_dict3', 'F')]
    mem_2 = BeliefState([],[], table_yaml)
    mem_2.belief = [(0.3, 'state_dict1', 'S'), (0.1, 'state_dict2', 'S'), (0.1, 'state_dict3', 'F')]

    combine(mem_1, mem_2, table_yaml)

def belief_state_equiv_test()

if __name__ == "__main__":

    # Want prints, make True!
    DEBUG = False

    # Get input table info; actions/conditions
    table_yaml = get_table_yaml()

    init_state = general_state_test(table_yaml)
    print("INIT STATE", init_state) # all statuses running
    print(init_state.state)

    init_state.updateState(['occluded','social_interaction'],['F','S'])
    print(init_state.state)

    init_belief_state = general_belief_state_test(init_state, table_yaml)
    print('before', init_belief_state.belief)

    after_action_belief_state = init_belief_state.apply_action_belief_state('move_toward')
    print('after', after_action_belief_state.belief)


    #combine_belief_states_test(table_yaml)
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

def state_equiv_test(init_state, table_yaml):

    print("##### Running state_equiv_test #####")

    # Testing state-state and belief_state-belief_state equivalence using new __eq__ funcs

    print("Testing state equivalence")
    # Make some different state instances
    state1 = State(table_yaml)
    state2 = State(table_yaml)
    state1.updateState(['occluded','social_interaction'],['F','S'])
    state2.updateState(['occluded','social_interaction'],['S','S'])

    print("Checking different states...")
    # Test they are different, meaning it should return false
    print("State1: ", state1.state)
    print("State2: ", state2.state)
    print(state1.__eq__(state2))

    print("Checking identical states...")
    # Test they are same
    state3 = State(table_yaml)
    state3.updateState(['occluded','social_interaction'],['F','S'])
    print("State1: ", state1.state)
    print("State3: ", state3.state)
    print(state1.__eq__(state3))

    # Test belief state stuff
    print("Testing belief state equivalence...")
    probs1 = [0.5,0.5]
    states1 = [state1,state2]
    probs2 = [0.5,0.5] #[0.5,0.9]
    states2 = [state1,state2] #[state1,state3]
    belief_state1 = BeliefState(states1,probs1, table_yaml)
    belief_state2 = BeliefState(states2,probs2, table_yaml)

    print(belief_state1.__eq__(belief_state2))

def combine_duplicates_test(belief_state, table_yaml):

    # Define list with duplicates to test
    state1 = State(table_yaml)
    state1.updateState(['occluded','social_interaction'],['F','S'])
    state2 = State(table_yaml)
    state2.updateState(['occluded','social_interaction'],['S','S'])
    state3 = State(table_yaml)
    state3.updateState(['occluded','social_interaction'],['F','S'])
    state4 = State(table_yaml)
    state4.updateState(['occluded','social_interaction'],['S','S'])

    # States 1 and 3 should be combined, states 2 and 4 also
    belief_list1 = [(0.1,state1,'S'),(0.3,state2,'S'),(0.3, state3,'S'),(0.3,state4,'S')]
    belief_list2 = [(0.5,state1,'S'),(0.5,state2,'S')]

    print('belief_list1, ',belief_list1)
    print('after1', belief_state.combine_duplicates(belief_list1))
    print('belief_list2, ',belief_list2)
    print('after2',belief_state.combine_duplicates(belief_list2))


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

    combine_belief_states_test(table_yaml)

    state_equiv_test(init_state, table_yaml)

    combine_duplicates_test(init_belief_state, table_yaml)
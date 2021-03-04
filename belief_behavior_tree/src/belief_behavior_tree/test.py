#!/usr/bin/env python

import yaml
import copy
from world_simulator.state import State
from belief_state import BeliefState, combine
from behavior_tree.behavior_tree import * #BehaviorTree, Sequence, ControlFlowNode
from refine_tree import *


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


def behavior_tree_test():

    #bt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/graeme.tree')
    #bt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant.tree')
    bt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_start.tree')

    # try this with belief behavior tree too
    
    print(bt.nodes)

    bt.print_BT()

    for node in bt.nodes:
        print(node.children)

    condition_to_resolve = 'child_moving_toward'
    parent = None

    for node in bt.nodes:
        if isinstance(node,ControlFlowNode):
            print(node, True)

            for child in node.children:
                if child.label == condition_to_resolve:
                    parent = node
                    #todo

        else:
            print(node, False)

    # Testing 
    bt.nodes = [Sequence()] + bt.nodes
    print(bt.nodes)
    bt.root = bt.nodes[0]
    bt.root.children = [bt.nodes[1]]

    bt.write_config('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_test.tree')


def children_dict(resolution_subtree_node_list):

    children = {}

    for i in range(len(new_node_list)):
        node = new_node_list[i]

        children[node] = []
        for j in range(len(new_node_list)):
            node2 = new_node_list[j]

            if isinstance(node2,ControlFlowNode): # flawed, what if a child is a control flow node????
                #finished adding children, at next 
                break

            elif node != node2:
                children[node].append(node2)

    pass



def update_bt(bt, condition_to_resolve, resolution_subtree_node_list):

    new_node_list = copy.deepcopy(bt.nodes)
    print(new_node_list)
    #new_bt = copy.deepcopy(bt) # this broke it!
    new_bt = bt

    # Check if at start, i.e. only one condition and no control nodes in tree
    if len(new_node_list) == 1:
        # Add root sequence
        new_node_list = [Sequence()] + new_node_list

    # not at start now, so find parent of condition, and add subtree to right of condition beneath parent 
    new_node_list = new_node_list + resolution_subtree_node_list
    new_bt.nodes = new_node_list
    print(new_node_list)

    children = {}


    for i in range(len(new_node_list)):

        node = new_node_list[i]

        if i == 0:
            new_bt.root = node

        if isinstance(node,ControlFlowNode):

            node.children = []
            for child in children(node):
                node.children.append(child)



    return new_bt

def update_bt_test():

    bt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_start.tree')
    condition_to_resolve = bt.nodes[0]
    test_new_subtree = [Sequence(),Condition('direct_social_interaction'),Action('bubbles')]

    new_bt = update_bt(bt, condition_to_resolve, test_new_subtree)
    
    for node in new_bt.nodes:
        print(node)

    print(new_bt.root)
    print(new_bt.root.children)

    new_bt.write_config('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_test.tree')



'''

def find_resolution_action(condition_to_resolve_string, action_table_list):

    resolution_action = None
    resolution_prob = 0

    for action_dict in action_table_list:

        new_resolution_found = False

        postconditions_list = action_dict['postconditions'] # [{0.5: [['child_moving_toward', 'S']]}, {0.5: [['child_moving_toward', 'F']]}]
        for i in range(len(postconditions_list)):
            print('i',i)
            prob_postconditions_dict = postconditions_list[i] # {0.5: [['child_moving_toward', 'S']]}
            print('prob_postconditions_dict', prob_postconditions_dict)
            prob = prob_postconditions_dict.keys()[0] # 0.5
            print('prob', prob)
            prob_postconditions_list = prob_postconditions_dict[prob] # [['child_moving_toward', 'S']]
            for postcondition_pair in prob_postconditions_list: # ['child_moving_toward', 'S']
                if postcondition_pair[0] == condition_to_resolve_string and postcondition_pair[1] == 'S':
                    # RESOLUTION FOUND (but is it the best?)
                    if resolution_action == None:
                        resolution_action = action_dict['action']
                        print('resolution_action 1',resolution_action, prob)
                        resolution_prob = prob
                        resolution_precondition_string_list = []
                        resolution_preconditions_pair_list = action_dict['preconditions']
                        for pair in resolution_preconditions_pair_list: # [direct_social_interaction, S]
                            resolution_precondition_string_list.append(pair[0])
                        new_resolution_found = True
                        break
                    elif prob > resolution_prob: #found better resolution
                        resolution_action = action_dict['action']
                        print('resolution_action after',resolution_action, prob)
                        resolution_prob = prob
                        new_resolution_found = True
                        break

            if new_resolution_found:
                # go look at next action
                break

    # return resolution action with highest probability of success based on postcondition probs (if multiple tie, first one chosen)
    return resolution_action, resolution_precondition_string_list


def generate_resolution_subtree(condition_to_resolve_string, resolution_action_string, resolution_precondition_string_list):

    # Input is string of condition we need to resolve, and string of action that can resolve it

    # Add children to root
    root = Sequence()
    condition_to_resolve = Condition(condition_to_resolve_string)
    root.children.append(condition_to_resolve)
    root.children.append(Fallback())

    # Add chilren to fallback
    # First preconditions
    for cond_string in resolution_precondition_string_list:
        root.children[1].children.append(Condition(cond_string))
    # Then action
    resolution_action = Action(resolution_action_string)
    root.children[1].children.append(resolution_action)

    print(root.children)
    print(root.children[1].children)

    return root
'''
if __name__ == "__main__":

    # Want prints, make True!
    DEBUG = False

    # # Get input table info; actions/conditions
    table_yaml = get_table_yaml()

    # yaml_test()

    init_state = general_state_test(table_yaml)
    # print("INIT STATE", init_state) # all statuses running
    # print(init_state.state)

    # init_state.updateState(['occluded','social_interaction'],['F','S'])
    # print(init_state.state)

    init_belief_state = general_belief_state_test(init_state, table_yaml)
    # print('before', init_belief_state.belief)

    # after_action_belief_state = init_belief_state.apply_action_belief_state('move_toward')
    # print('after', after_action_belief_state.belief)

    # combine_belief_states_test(table_yaml)

    # state_equiv_test(init_state, table_yaml)

    # combine_duplicates_test(init_belief_state, table_yaml)

    #behavior_tree_test() # general brainstorm testing

    #update_bt_test() #unfinished

    #generate_resolution_subtree()

    resolution_action, resolution_precondition_string_list = find_resolution_action('child_moving_toward',init_belief_state.action_table_list)
    print(resolution_action)
    print(resolution_precondition_string_list)

    '''
    bt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_start.tree')
    resolution_subtree_root = generate_resolution_subtree('child_moving_toward','bubbles',['direct_social_interaction'])
    print(resolution_subtree_root)
    bt.root = resolution_subtree_root
    bt.write_config('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_test.tree')
    
    
    condition_to_resolve_string = 'child_moving_toward'
    resolution_subtree = resolution_subtree_root
    bt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_start.tree')
    bt.root = traverse_and_replace(bt.root, condition_to_resolve_string, resolution_subtree)

    bt.write_config('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_test.tree')
    '''
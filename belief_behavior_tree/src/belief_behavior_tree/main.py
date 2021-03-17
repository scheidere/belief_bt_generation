#!/usr/bin/env python

import yaml
import copy
#from infant_simulator.state import WorldState
from infant_simulator.run_simulator import *
#from world_simulator.state import State
from belief_state import BeliefState, combine
from behavior_tree.behavior_tree import * #BehaviorTree, Sequence, ControlFlowNode
from refine_tree import *
from behavior_tree.belief_behavior_tree import *


def get_table_yaml():
    # Read yaml file
    with open('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/belief_behavior_tree/src/input/action_tables.yaml', 'r') as stream:
        try:
            #print(yaml.safe_load(stream))
            table_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    return table_yaml

def yaml_test():

    # Read yaml file
    with open('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/belief_behavior_tree/src/input/action_tables.yaml', 'r') as stream:
        try:
            #print(yaml.safe_load(stream))
            action_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


    print('TEST', action_yaml['action_table'])

    print('+++++++++++++++++++')

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


if __name__ == '__main__':
    #yaml_test()

    num_iterations = 0 # need to stop at 900
    step_size = 1 # Only run each bt one iteration in the world before reevaluating
    goal_prob = 0.9 # We may need to disable this because we aren't stopping once the goal condition is reached
    current_score = 0
    current_distance = 0

    table_yaml = get_table_yaml()
    #print('yaml', len(table_yaml))

    # Get initial bbt/bt
    #bbt = BehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_start.tree')
    bbt = BeliefBehaviorTree('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/behavior_tree/config/infant_start.tree')
    #bbt = get from bbt method

    rospy.init_node('infant_simulator')
    sim = InfantSimulator(bt = bbt)

    while not rospy.is_shutdown() and num_iterations < 1:
        print('BT node list: ', sim.bt.nodes)

        current_score, current_distance, state = sim.run_sim(table_yaml, step_size, current_score, current_distance) #calls controller.run()

        # Current belief state is physical state with prob 1 (Really this simple...???)
        current_belief_state = BeliefState([state], [1], table_yaml)
        print("Belief state: ", current_belief_state.belief)
        print('State in belief state: ', current_belief_state.belief[0][1].state)

        #update bt using bbt method
        bbt = refine_tree(bbt, goal_prob, current_belief_state)

        sim.update_bt(bbt)
    
        num_iterations +=1





# move
# update belief (with info from world not func within refine tree)
# refine tree
# loop


#??? write this based on pseudocode in interim paper
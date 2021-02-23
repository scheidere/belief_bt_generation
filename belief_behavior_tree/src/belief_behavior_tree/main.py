#!/usr/bin/env python

import yaml


def yaml_test():

    # Read yaml file
    with open('/home/scheidee/belief_behavior_tree_ws/src/belief_bt_generation/belief_behavior_tree/src/input/action_tables.yaml', 'r') as stream:
        try:
            #print(yaml.safe_load(stream))
            action_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

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
    yaml_test()


# move
# update belief (with info from world not func within refine tree)
# refine tree
# loop
#!/usr/bin/env python

import copy
from belief_state import BeliefState

#import ReturnStatus ???

#from behavior_tree.belief_behavior_tree import Condition 
from behavior_tree.behavior_tree import *

'''
Planning routine that expands the tree accoding to the belief state, using template methods

'''

def refine_tree(behavior_tree, goal_prob, current_belief_state):
    '''

    # This is bbt algorithm 3 #

    Input:
    - behavior_tree: the current behavior tree that just was executed in the world
    - goal_prob: the probability threshold, 
    i.e. confidence we must have that we have reached the goal before stopping
    - current_belief_state: the belief state calculated from the world after 
    the current behavior tree was executed in the world

    Output:
    - new behavior tree
    '''

    belief_state = copy.deepcopy(current_belief_state)

    # Initialize probability that you are in the goal state (i.e. all goal conditions satisfied)
    prob_in_goal_state = 0

    # Expand and test tree until
    # the likelihood it can allow the robot to reach the goal state
    # is achieved beyond a certain threshold, goal_prob
    while prob_in_goal_state < goal_prob:

        # Find the most probable deepest "failing" condition (failing or running)
        condition_to_resolve = find_failed_condition(behavior_tree, belief_state) #???

        '''
        if is_threatened(condition_to_resolve): ???

            # If you can resolve the threat by rearranging the tree, do so
            behavior_tree = resolve_threat(behavior_tree, belief_state, condition_to_resolve) ???

        else:

            # Otherwise, resolve by adding a necessary subtree around that condition
            behavior_tree = resolve_by_insert(behavior_tree, belief_state, condition_to_resolve) ???
        '''

        # Skipping resolve by threat part because I have no idea how it works...
        # This method expands according to the most probable deepest failing condition and its relationship to postconditions in the yaml table
        behavior_tree = resolve_by_insert(behavior_tree, belief_state, condition_to_resolve)

        # Update belief state now that the current behavior tree has changed (but only within this algorithm)
        belief_state = self_simulate(behavior_tree, current_belief_state)

        # Differentiate between the states where the goal has been reached, and all others
        # Update current belief_state with remaining states
        #belief_state_goal, belief_state = belief_state.split_by_return_status(behavior_tree, return_status = ReturnStatus.SUCCESS)
        belief_state_goal, belief_state = belief_state.split_by_return_status(return_status = ReturnStatus.SUCCESS, check_goal_condition=True)

        # Calculate the probability that the actual physical goal state has been reached via belief state
        prob_in_goal_state = belief_state_goal.probability_goal_reached() #???


    # Return the final expanded behavior tree
    return behavior_tree


def find_failed_condition(behavior_tree, mem):

    # IF A = root, B, C are its children, and DE are B's children, F is Cs, we want nodes = [A,B,C,D,E,F] (bc same level, def same level)
    nodes = []

    # Setup a stack data structure (similar to nodes_worklist)
    nodes_stack = []
    nodes_stack.append(behavior_tree.root) #push

    # Do the traversal, using the stack to help
    while len(nodes_stack) != 0:
        # Pop first element
        current_node = nodes_stack.pop(0)

        nodes.append(current_node)
        for child_idx in range(len(current_node.children)):
            nodes_stack.append(current_node.children[child_idx])

    # Want to look at deepest first
    nodes = reversed(nodes) # [F,E,D,C,B,A]

    deepest_failed_condition_probs = [0]*len(nodes)
    # Evaluate for all states in belief state
    for i in range(len(mem.belief)):
        state = mem.belief[i][1]

        # Only check "failure" of conditions currently in behavior tree
        for j in range(len(nodes)):
            node = nodes[j]
            condition_string = node.label
            status = state.state[condition_string]

            if isinstance(node, Condition) and not evaluate(condition_string,status): #??? eval function incomplete

                # First time we find failing condition is inherently the deepest failing condition due to order of nodes list

                # Add state probability to deepest failing condition index in given state
                deepest_failed_condition_probs[j] += mem.belief[i][0]
                break
                

    #max_prob = max(deepest_failed_condition_probs)
    #max_index = deepest_failed_condition_probs.index(max_prob)

    max_prob = 0
    for i in range(len(deepest_failed_condition_probs)):
        prob = deepest_failed_condition_probs[i]
        if prob > max_prob:
            max_prob = prob
            max_index = i

    # Return most probable deepest failing condition
    return nodes[i]




def evaluate(condition_string, status):

    # Example:
    # condition_string = 'child_moving_toward'
    # status = 'R'

    # NOTE: not sure how to deal with decorator addition in yaml or anywhere else so I am assuming all goal condition statuses are 'S' 
    # i.e. instead of occluded = 'F' for bubbles, it would be not_occluded = 'S'
    # Yes, this is redundant to an extent, but I needed a temporary simple solution

    # Future solution:
    # if status = 'F' or = 'S' under decorator or = 'R' anywhere:
    #   return False

    # return True

    if status == 'F' or status == 'R':
        return False

    return True




def resolve_by_insert(behavior_tree, belief_state, condition_to_resolve_string):
    #???
    ##pass

    # Get bt node list in form that tracks parent/children easily

    # find action that is most likely to resolve condition 

    # insert that action beneath a sequence to the right of the condition
    # with preconditions to the left

    ###return updated behavior tree (which is actually a bbt)

    ####check if parent is a root -> set tree.root = new parent


    action_table_list = belief_state.action_table_list

    resolution_action_string, resolution_precondition_string_list = find_resolution_action(condition_to_resolve_string, action_table_list)

    resolution_subtree = generate_resolution_subtree(condition_to_resolve_string,resolution_action_string,resolution_precondition_string_list)

    # Check if we are at start, i.e. only goal condition in tree, if so add sequence root
    # if isinstance(behavior_tree.root,Condition):
    #     old_root = behavior_tree.root
    #     behavior_tree.root = Sequence()
    #     behavior_tree.root.children.append(old_root)

    # else:
    #     behavior_tree.root = traverse_and_replace(behavior_tree.root, condition_to_resolve_string,resolution_subtree)

    # Because of the way the generate_resolution subtree works, a sequence root gets added automatically to replace exact position of each condition that is resolved
    behavior_tree.root = traverse_and_replace(behavior_tree.root, condition_to_resolve_string,resolution_subtree)

    return behavior_tree

def traverse_and_replace(root, condition_to_resolve_string, resolution_subtree):

    #not tested yet, double check logic ???

    # Check if at start, i.e. root is goal condition
    if root.label == condition_to_resolve_string:
        return resolution_subtree # includes root sequence, condition, and fallback subtree with preconditions and action
    else:
        for i in range(len(root.children)):
            node = root.children[i]
            if node.label == condition_to_resolve_string:
                root.children[i] = resolution_subtree
                return root
            else:
                return traverse_and_replace(node,condition_to_resolve_string, resolution_subtree)




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
                        resolution_precondition_string_list = []
                        resolution_preconditions_pair_list = action_dict['preconditions']
                        for pair in resolution_preconditions_pair_list: # [direct_social_interaction, S]
                            resolution_precondition_string_list.append(pair[0])
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

    return root # subtree contained within root in parent/children storage



def is_threatened(condition_to_resolve):

    #return True if is threatened 
    pass

def resolve_threat():

    # ???
    pass
#!/usr/bin/env python

import copy
from behavior_tree.belief_state import BeliefState

#import ReturnStatus ???

#from behavior_tree.belief_behavior_tree import Condition 
#from behavior_tree.behavior_tree import *
from behavior_tree.belief_behavior_tree import *
from self_simulation import self_simulate

'''
Planning routine that expands the tree accoding to the belief state, using template methods

'''

def refine_tree(behavior_tree, goal_prob, current_belief_state, table_yaml, do_not_reuse_action_row_nums):
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

    max_ticks = 100

    belief_state = copy.deepcopy(current_belief_state)

    # Initialize probability that you are in the goal state (i.e. all goal conditions satisfied)
    prob_in_goal_state = 0

    count = 0

    # Expand and test tree until
    # the likelihood it can allow the robot to reach the goal state
    # is achieved beyond a certain threshold, goal_prob
    while prob_in_goal_state < goal_prob:

        print('Iteration', count)

        print('root 2', behavior_tree.root)

        # Find the most probable deepest "failing" condition (failing or running)
        condition_to_resolve = find_failed_condition(behavior_tree, belief_state) #???

        # If no condition to resolve, return current tree
        if condition_to_resolve == None:
            #pub.publish('None')
            return behavior_tree, do_not_reuse_action_row_nums

        #else:
        #    pub.publish(condition_to_resolve.label)

        '''
        if is_threatened(condition_to_resolve): ???

            # If you can resolve the threat by rearranging the tree, do so
            behavior_tree = resolve_threat(behavior_tree, belief_state, condition_to_resolve) ???

        else:

            # Otherwise, resolve by adding a necessary subtree around that condition
            behavior_tree = resolve_by_insert(behavior_tree, belief_state, condition_to_resolve) ???
        '''
        print('Post resolution bt root 1', behavior_tree.root)
        # Skipping resolve by threat part because I have no idea how it works...
        # This method expands according to the most probable deepest failing condition and its relationship to postconditions in the yaml table
        behavior_tree, do_not_reuse_action_row_nums = resolve_by_insert(behavior_tree, belief_state, condition_to_resolve, do_not_reuse_action_row_nums)

        print('Post resolution bt root 2', behavior_tree.root)

        # Update belief state now that the current behavior tree has changed (but only within this algorithm)
        belief_state = self_simulate(behavior_tree, current_belief_state, max_ticks, table_yaml)

        # Differentiate between the states where the goal has been reached, and all others
        # Update current belief_state with remaining states
        #belief_state_goal, belief_state = belief_state.split_by_return_status(behavior_tree, return_status = ReturnStatus.SUCCESS)
        belief_state_goal, belief_state = belief_state.split_by_return_status(return_status = 'S', check_goal_condition=True)

        # Calculate the probability that the actual physical goal state has been reached via belief state
        prob_in_goal_state = belief_state_goal.probability_goal_reached() #???

        count += 1
    # Return the final expanded behavior tree
    return behavior_tree, do_not_reuse_action_row_nums


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

        print('root 1', behavior_tree.root)

        behavior_tree.generate_nodes_list()
        behavior_tree.print_BT()

        nodes.append(current_node)
        for child_idx in range(len(current_node.children)):
            nodes_stack.append(current_node.children[child_idx])

    #print('NODES 1', nodes)

    # Want to look at deepest first
    nodes = list(reversed(nodes)) #reversed(nodes) # [F,E,D,C,B,A]

    #print('NODES', nodes)

    deepest_failed_condition_probs = [0]*len(nodes)
    # Evaluate for all states in belief state
    for i in range(len(mem.belief)):
        state = mem.belief[i][1]

        #print('=========================== 1')

        # Only check "failure" of conditions currently in behavior tree
        for j in range(len(nodes)):
            node = nodes[j]
            condition_string = node.label

            #print('===========================', condition_string)

            if isinstance(node, Condition):
                status = state.state[condition_string]
                if not evaluate(condition_string,status): # if eval returns false, condition is failing

                    #print('SHOULD BE A CONDITION', condition_string)

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
    if max_prob: # max_prob not zero
        return nodes[max_index]
    return None #no failed conditions




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




def resolve_by_insert(behavior_tree, belief_state, condition_to_resolve, do_not_reuse_action_row_nums):
    #???
    ##pass

    # Get bt node list in form that tracks parent/children easily

    # find action that is most likely to resolve condition 

    # insert that action beneath a sequence to the right of the condition
    # with preconditions to the left

    ###return updated behavior tree (which is actually a bbt)

    ####check if parent is a root -> set tree.root = new parent

    pub = rospy.Publisher('deepest_failing_condition', String, queue_size=10)

    condition_to_resolve_string = condition_to_resolve.label

    action_table_list = belief_state.action_table_list

    resolution_action_string, resolution_precondition_string_list, used_action_num = find_resolution_action(condition_to_resolve_string, action_table_list, do_not_reuse_action_row_nums)

    if used_action_num != None:

        pub.publish(condition_to_resolve.label + ' needs ' + resolution_action_string)

        do_not_reuse_action_row_nums.append(used_action_num)

        resolution_subtree = generate_resolution_subtree(condition_to_resolve_string,resolution_action_string,resolution_precondition_string_list)
        pub.publish('resolution subtree starts with '+ resolution_subtree.label + ' and ' + resolution_subtree.children[0].label)
        # Check if we are at start, i.e. only goal condition in tree, if so add sequence root
        # if isinstance(behavior_tree.root,Condition):
        #     old_root = behavior_tree.root
        #     behavior_tree.root = Sequence()
        #     behavior_tree.root.children.append(old_root)

        # else:
        #     behavior_tree.root = traverse_and_replace(behavior_tree.root, condition_to_resolve_string,resolution_subtree)

        print('Post resolution bt root 1 (in resolve by insert)', behavior_tree.root)
        # Because of the way the generate_resolution subtree works, a sequence root gets added automatically to replace exact position of each condition that is resolved
        behavior_tree.root = traverse_and_replace(behavior_tree.root, condition_to_resolve_string,resolution_subtree)
        print('Post resolution bt root 2 (in resolve by insert)', behavior_tree.root)

    else:
        if resolution_action_string != None:
            pub.publish(condition_to_resolve.label + ' cannot be resolved but attempted ' + resolution_action_string + ' and num is ' + str(used_action_num)) # + str(resolution_precondition_string_list))
        else:
            pub.publish(condition_to_resolve.label + ' cannot be resolved but attempted ' + 'None' + ' and num is ' + str(used_action_num)) # + str(resolution_precondition_string_list))

    pub.publish('Used actions list' + str(do_not_reuse_action_row_nums))
    return behavior_tree, do_not_reuse_action_row_nums

def traverse_and_replace(root, condition_to_resolve_string, resolution_subtree):

    #not tested yet, double check logic ???

    # ??? may need to make this so that all instances of condition to resolve are resolved not just first

    print('root in traverse and replace', root)

    # Check if at start, i.e. root is goal condition
    if root.label == condition_to_resolve_string:
        return resolution_subtree # includes root sequence, condition, and fallback subtree with preconditions and action
    else:
        for i in range(len(root.children)):
            node = root.children[i]
            if node.label == condition_to_resolve_string:
                root.children[i] = resolution_subtree
                break

            else:
                traverse_and_replace(node,condition_to_resolve_string, resolution_subtree)
                

        return root


def find_resolution_action(condition_to_resolve_string, action_table_list, do_not_reuse_action_row_nums):

    print('======== find_resolution_action function ========')

    print('condition_to_resolve_string', condition_to_resolve_string)

    resolution_action = None
    resolution_prob = 0
    resolution_precondition_string_list = None

    found_prev_used_action = False

    row = 0
    for action_dict in action_table_list:

        new_resolution_found = False

        

        cumulative_prob_action_will_resolve = 0

        postconditions_list = action_dict['postconditions'] # [{0.5: [['child_moving_toward', 'S']]}, {0.5: [['child_moving_toward', 'F']]}]
        for i in range(len(postconditions_list)):
            #print('i',i)
            prob_postconditions_dict = postconditions_list[i] # {0.5: [['child_moving_toward', 'S']]}
            #print('prob_postconditions_dict', prob_postconditions_dict)
            prob = prob_postconditions_dict.keys()[0] # 0.5
            #print('prob', prob)
            prob_postconditions_list = prob_postconditions_dict[prob] # [['child_moving_toward', 'S']]
            for postcondition_pair in prob_postconditions_list: # ['child_moving_toward', 'S']
                if postcondition_pair[0] == condition_to_resolve_string and postcondition_pair[1] == 'S':
                    # RESOLUTION FOUND (but is it the best?)
                    cumulative_prob_action_will_resolve += prob

        if cumulative_prob_action_will_resolve > 0: # Found action with nonzero prob of resolution
        
            # If you havent already used the action in current action_dict (row of yaml table) to expand tree, you can use it now
            if not (row in do_not_reuse_action_row_nums):

                # Therefore, save action (if first or if better than previous)       
                if resolution_action == None or cumulative_prob_action_will_resolve > resolution_prob:
                    #print('Resolution action is None', resolution_action)
                    resolution_action = action_dict['action']
                    used_action_num = row
                    #print('Resolution action updated first time', resolution_action)
                    #print('resolution_action 1',resolution_action, prob)
                    resolution_prob = cumulative_prob_action_will_resolve
                    resolution_precondition_string_list = []
                    resolution_preconditions_pair_list = action_dict['preconditions']
                    for pair in resolution_preconditions_pair_list: # [direct_social_interaction, S]
                        resolution_precondition_string_list.append(pair[0])
                    new_resolution_found = True


                # elif cumulative_prob_action_will_resolve > resolution_prob: #found better resolution
                #     resolution_action = action_dict['action']
                #     used_action_num = row
                #     #print('resolution_action after',resolution_action, prob)
                #     resolution_prob = cumulative_prob_action_will_resolve
                #     resolution_precondition_string_list = []
                #     resolution_preconditions_pair_list = action_dict['preconditions']
                #     for pair in resolution_preconditions_pair_list: # [direct_social_interaction, S]
                #         resolution_precondition_string_list.append(pair[0])
                #     new_resolution_found = True
                #     break
            
            else:
                found_prev_used_action = True



        row += 1 

    print('resolution action', resolution_action)
    print('resolution_precondition_string_list', resolution_precondition_string_list)
    print('======== find_resolution_action function END ========')
    # return resolution action with highest probability of success based on postcondition probs (if multiple tie, first one chosen)

    if not resolution_action:
        print('USED ACTION ROW NUMS', do_not_reuse_action_row_nums)
        if not found_prev_used_action: # only crash if we didnt find a resolution at all (finding one already used is okay)
            raise Exception("Can't find action to resolve following condition",condition_to_resolve_string)

        else:
            return resolution_action, resolution_precondition_string_list, None
        print('NO RESOLUTION ACTION FOUND... This should not happen.')
        print('Condition attempted to resolve: ', condition_to_resolve_string)

    return resolution_action, resolution_precondition_string_list, used_action_num


def generate_resolution_subtree(condition_to_resolve_string, resolution_action_string, resolution_precondition_string_list):

    # Input is string of condition we need to resolve, and string of action that can resolve it

    # Add children to root
    root = Fallback()
    condition_to_resolve = Condition(condition_to_resolve_string)
    root.children.append(condition_to_resolve)
    
    # IS THIS RIGHT? SEQUENCE OR FALLBACK?
    #root.children.append(Fallback())
    root.children.append(Sequence())

    # Add chilren to fallback
    # First preconditions
    for cond_string in resolution_precondition_string_list:
        root.children[1].children.append(Condition(cond_string))
    # Then action
    resolution_action = Action(resolution_action_string)
    root.children[1].children.append(resolution_action)

    print('NEW ROOT children+++++++++++++++++++++++++++++++++', root.children)
    print(root.children[1].children)

    return root # subtree contained within root in parent/children storage



def is_threatened(condition_to_resolve):

    #return True if is threatened 
    pass

def resolve_threat():

    # ???
    pass
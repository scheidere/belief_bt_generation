#!/usr/bin/env python

import copy

from world_simulator.state import State


class BeliefState:
    def __init__(self, states, probabilities, table_yaml, DEBUG = False):

        # states and probabilities expected in a list format

        # Initialize info tuple list, i.e. list of tuples
        self.belief = [] #changed mem to belief

        # Initial belief state (e.g. current physical state with prob 1)
        self.initBeliefState(states,probabilities)

        ######self.action_postcondition_probs = pull table info from action yaml

        self.table_yaml = table_yaml
        self.action_table_list = self.table_yaml['action_table']
        #print(self.action_table_list)

        self.DEBUG = DEBUG

    def initBeliefState(self, states, probabilities):

        # states and probabilities expected in a list format

        for i in range(len(states)):

            state = states[i]
            prob = probabilities[i]
            #actions = [] # will get updated later
            return_status = None #???

            #subset = (prob, state)
            # actions denotes the active actions in the given state, will be search to determine if any are delayed i.e. RUNNING ???
            #subset = (prob, state, actions)
            subset = (prob, state, return_status)

            self.belief.append(subset)

    def check_preconditions(self, preconditions):

        # Input will be a list of lists, each list contains a precondition string and 'S', 'F', or 'R'

        # Look through each physical state and determine whether input preconditions are satisfied

        # Initialize tracker of len = num of states in belief state
        # Keep track of which state has satisfied preconditions
        satisfied_list = [None]*len(self.belief)

        # Walk through states in belief state
        for i in range(len(self.belief)):
            state = self.belief[i][1].state
            if self.DEBUG:
                print('state: ', state)


            preconditions_hold = None
            # Walk through conditions in preconditions list
            for j in range(len(preconditions)):
                if self.DEBUG:
                    print('precondition: ', preconditions[j])
                condition_string = preconditions[j][0]
                condition_status = preconditions[j][1] # 'R', 'F', 'S'

                # Check for precondition satisfaction, and track conclusion
                if state[condition_string] == condition_status: 
                    preconditions_hold = True
                else:
                    # One precondition does not hold so current state fails, add False to satisfied list
                    preconditions_hold = False
                    break


            if preconditions_hold:
                satisfied_list[i] = True
            else:
                satisfied_list[i] = False

        return satisfied_list


    def generate_matched_state_action_list(self, action):

        # Given an action string

        # Return a list of nums that each represent action/precondition set needed for the state at each index

        # e.g. if there are three states, and two move_toward actions
        # if the first move_toward preconditions are satisfied in states 1 and 2, and the second in state 3
        # tracker = [0,0,1]
        # This tells us when we tick the input action, if in states 1 and 2 use action 0, and otherwise action 1


        # Track which action instance in the table has satisfied preconditions for each state
        tracker = [-1]*len(self.belief)

        # Look at each action
        for action_index in range(len(self.action_table_list)):

            action_info_dictionary = self.action_table_list[action_index]
            if self.DEBUG:
                print(action_info_dictionary)

            # Check if action is the right one
            key = action_info_dictionary.keys()[0]
            #print(key)
            current_action = action_info_dictionary[key]
            #print(current_action)
            if current_action == action:

                if self.DEBUG:
                    print('samesies')

                # Get preconditions
                preconditions = action_info_dictionary['preconditions'] # list of lists
                if self.DEBUG:
                    print(preconditions)
                precondition_satisfaction_list = self.check_preconditions(preconditions) # list of either True or False for each state

                # Update tracker
                for i in range(len(tracker)):

                    if precondition_satisfaction_list[i] == True and tracker[i] == -1:
                        tracker[i] = action_index

        if self.DEBUG:
            print('TRACKER', tracker)
        return tracker



    def combine_duplicates(self, new_belief):

        # Input is a belief state list (same type as self.belief)
        # Search for states that are the same
        '''
        not done 

        no_duplicates_belief = copy.deepcopy()
        
        for state_info_tuple_1 in new_belief:
            for state_info_tuple_2 in new_belief:
                

        return new_belief
        '''
        pass

    def apply_action_belief_state(self, action):

        print("##### Running apply_action_belief_state #####")

        ##redo given new yaml syntax

        # Initialize new belief state list
        new_belief = []

        # Match action index of satisfied preconditions with associated state index
        state_action_match_list = self.generate_matched_state_action_list(action)

        # Apply action (according to above list) for each state
        for i in range(len(state_action_match_list)):

            # Get state info from belief state
            prob = self.belief[i][0]
            state = self.belief[i][1]

            # Get which action you need (i.e. which preconds satisfied)
            action_index = state_action_match_list[i]
            action_info_dictionary = self.action_table_list[action_index]


            postconditions_list = action_info_dictionary['postconditions']
            if self.DEBUG:
                print('postconditions_list', postconditions_list)
            for i in range(len(postconditions_list)):
                post_prob_dict = postconditions_list[i]
                post_prob = post_prob_dict.keys()[0]
                if self.DEBUG:
                    print('post_prob_dict', post_prob_dict)
                    print('post_prob', post_prob)
                new_state = copy.deepcopy(state)
                for postcondition in post_prob_dict[post_prob]:
                    if self.DEBUG:
                        print(postcondition)

                    # Update condition to post status
                    new_state.state[postcondition[0]] = postcondition[1]

                    # Update likelihood of subsequent state
                    new_prob = prob*post_prob

                    # Add new prob, state info to new belief state list
                    new_belief.append((new_prob, new_state, 'S')) # S because action succeeded???

                if self.DEBUG:
                    print('NEW STATE',new_state.state)
        # Update actual belief state, combining probs if same state was added twice via postcondition updates
        #self.belief = self.combine_duplicates(new_belief) ???
        self.belief = new_belief

        return self

    def get_condition_groups(self, postconditions):

        temp_postconds = copy.deepcopy(postconditions)

        for postcondition in postconditions:

            string = postconditions[1]

    '''

    def split_by_delayed_actions(self, active_bt_actions): # need return statuses included?
        # Given belief state, whole or subset
        # Depending on status returned, updated belief state, i.e. mem
        return mem_given_return_status # maybe return is more complex

        
        do we need action info???

        need to split by which actions are running and which are S/F -> only need to know status of root node (it will return running if any action running)

        "return result.action_key not in s or len(s[result.action_key]) == 0" ??? #see planningbehaviortree.py/bs_interface.py from their code

    def split_by_return_status(self, return_status = None):
        # Given belief state, whole or subset
        # Depending on status returned, updated belief state, i.e. mem
        return mem_given_return_status # maybe return is more complex

        Example: return status = ReturnStatus.SUCCESS

        Find all physical states in belief state which have ReturnStatus.SUCCESS for given node

        Determine for that node in each state, what its status is after checking condition statuses in State, given current_bt

        do we need action info???


    def apply_delayed_actions(self):

        # Initialize empty resulting belief state
        resulting_mem = BeliefState([],[])

        # Pull action table info from yaml
        ???


        # For states in current belief state, check for delayed actions
        for i in range(len(self.belief)):
            actions = self.belief[i][2]
            if actions: # Check if state i has active actions
                for action in actions:
                    # For a given action, differentiated by precondtions if necessary, get probabilistic outcome
                    outcome = ??? from postcondtions

                    # Update resulting belief state with potential outcomes
                    resulting_mem.update???

        return resulting_mem


    def probability_goal_reached(self):
        return single prob combining all state probs in belief state
    '''

def combine (mem_1, mem_2, table_yaml, DEBUG = False):

    '''
    Combines two belief states.

    If a physical state is in both mem 1 and 2, the probs are added for new mem

    The resulting probs should still add to one 
    because of how the split function works in alg 2, i.e. self_simulation

    Revist to optimize: compare slices instead of reassigning, etc

    '''

    new_mem = BeliefState([],[], table_yaml)

    if DEBUG:
        print('1', mem_1.belief)
        print('2', mem_2.belief)
        print('new', new_mem.belief)

    for state_info_tuple_1 in mem_1.belief:

        is_duplicate = False

        state_1 = state_info_tuple_1[1:] #includes root return status, should be same for identical states

        if DEBUG:
            print('state_1', state_1)

        for state_info_tuple_2 in mem_2.belief:

            state_2 = state_info_tuple_2[1:] #includes root return status, should be same for identical states

            if DEBUG:
                print('state_2', state_2)

            # Check that states are the same, including return status
            if state_1 == state_2:

                if DEBUG:
                    print("duplicate found", state_1, state_2)

                is_duplicate = True

                # If the belief states you are combing contain the same state, add the probs
                new_prob = state_info_tuple_1[0] + state_info_tuple_2[0]

                # Create the new tuple for that state
                new_state_info_tuple = (new_prob, state_1[0], state_1[1])

                # Add state found in both mems to new mem with updated prob
                new_mem.belief.append(new_state_info_tuple)
                if DEBUG:
                    print('new yes dup', new_mem.belief)


                # Remove from mem_2
                mem_2.belief.remove(state_info_tuple_2)

                # Duplicate of mem_1 state found so no need to continue searching mem_2
                break

        if not is_duplicate:
            # info from 1 not in 2 so add info 1 by itself, unchanged
            new_mem.belief.append(state_info_tuple_1)

            if DEBUG:
                print('new not dup', new_mem.belief)

    if DEBUG:
        print('done with mem_1')

        print('mem_2 after dups', mem_2.belief)

    # Add remaining state tuples that were not also in mem_1
    for state_info_tuple_2 in mem_2.belief:
        new_mem.belief.append(state_info_tuple_2)

    if DEBUG:
        print('new mem', new_mem.belief)

    return new_mem
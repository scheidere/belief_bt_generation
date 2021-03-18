#!/usr/bin/env python

from behavior_tree.belief_state import BeliefState, combine



def self_simulate(behavior_tree, mem, max_ticks, table_yaml): #, max_states): #assuming input bt is same as bbt, i.e. instance of bbt class
    '''
	# This is bbt algorithm 2 #

	Input:
	- behavior_tree: current behavior tree, i.e. belief behavior tree, as expanded in refine_tree (alg 3)
	- mem: current belief state within expansion algorithm, refine_tree

	Output:
	- Estimated, i.e. self-simulated belief state that would occur if that behavior tree were deployed
    '''

    # Initialize resulting belief state
    results = BeliefState([],[], table_yaml)

    # Do while input belief state is not empty
    tick_count = 0
    while mem.belief and tick_count < max_ticks: #??? also need to limit by num states in memory

    	# Update belief state via a tick of the tree, new belief state denoted as current
    	current_mem = behavior_tree.tick_mem(mem) 
    	tick_count += 1

    	# Split the belief state, mem, based on which actions have completed in each state (ended), 
    	# and which have been delayed because concurrent actions aren't allowed (those states retained in mem)
    	#ended, mem = current_mem.split_by_delayed_actions(behavior_tree.active_actions)
    	ended, mem = current_mem.split_by_delayed_actions()

    	# Update results with states that have no delayed actions, i.e. only actions that have ended
    	results = combine(results, ended)
    	###print("Results (do they need to be normalized?): ", results.belief)

    	# Now deal with the actions that could not be completed in previous tick, i.e. the delayed actions
    	###mem = mem.apply_delayed_actions() ??? not sure how this works yet


    # Once all delayed actions have been delt with, i.e. mem is empty, return results, i.e. expected belief state
    return results
	





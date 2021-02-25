#!/usr/bin/env python

import copy
from belief_state import BeliefState

import ReturnStatus ???

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
		condition_to_resolve = find_failed_condition(behavior_tree, belief_state)

		if is_threatened(condition_to_resolve):

			# If you can resolve the threat by rearranging the tree, do so
			behavior_tree = resolve_threat(behavior_tree, belief_state, condition_to_resolve)

		else:

			# Otherwise, resolve by adding a necessary subtree around that condition
			behavior_tree = resolve_by_insert(behavior_tree, belief_state, condition_to_resolve)


		# Update belief state now that the current behavior tree has changed (but only within this algorithm)
		belief_state = self_simulate(behavior_tree, current_belief_state)

		# Differentiate between the states where the goal has been reached, and all others
		# Update current belief_state with remaining states
		#belief_state_goal, belief_state = belief_state.split_by_return_status(behavior_tree, return_status = ReturnStatus.SUCCESS)
		belief_state_goal, belief_state = belief_state.split_by_return_status(return_status = ReturnStatus.SUCCESS)

		# Calculate the probability that the actual physical goal state has been reached via belief state
		prob_in_goal_state = belief_state_goal.probability_goal_reached()


	# Return the final expanded behavior tree
	return behavior_tree


def is_threatened(condition_to_resolve):

	return True if is threatened 

def find_failed_condition(behavior_tree, mem):

	# Get node list from behavior_tree object
	nodes = behavior_tree.nodes

	for i in range(len(mem.belief)):
		state = mem.belief[i][1]


	find most probable (use belief state) deepest (level in behavior tree)
	???


def resolve_threat():
	???


def resolve_by_insert():
	???

#!/usr/bin/env python

import rospy
import yaml
import rospkg
from cfg import CFG, Word, Character, createWord, filterDuplicates, exportBT, extract_subtrees

from behavior_tree.behavior_tree import *
from behavior_tree_msgs.msg import Status, Active

import behavior_tree.behavior_tree_graphviz as gv
import zlib

def init_bt(bt):
    # print("BT_Interface initialising BT...")
    for node in bt.nodes:
        node.init_ros()
        # print(node.label)
    # print("BT finished init")

def tick_bt(bt):
    bt.tick()#root.tick(True)

    source = gv.get_graphviz(bt)
    source_msg = String()
    source_msg.data = source
    graphviz_pub.publish(source_msg) 

    compressed = String()
    compressed.data = zlib.compress(source)
    compressed_pub.publish(compressed)

if __name__ == '__main__':

	# Initialise the node
    rospy.init_node('show_tree')
    # Get the config file etc
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('simulator') + "/config/" + rospy.get_param('~config')
    with open(filepath, 'r') as stream:
        config = yaml.safe_load(stream)
    robot_id = rospy.get_param('~robot_id')
    num_robots = rospy.get_param('~num_robots')
    seed = rospy.get_param('~seed')

    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    try:

        # Setup a simple BT
        #character_list = [Character('?'),Character('('), Character('->'),Character('('),\
            #Character('(target_found_90)'),Character('?'),Character('('),Character('(in_comms)'),\
            #Character('[go_to_comms]'),Character(')'),Character('[report]'),Character(')'),Character('[go_to_belief]'),Character(')')]
        
        #test
        #character_list = [Character('->'),Character('('),Character('[report]'),Character('[go_to_belief]'),Character('[random_walk]'),Character(')')]

        #single robot single target mcts generated tree - not great
        #character_list = [Character('->'),Character('('),Character('[go_to_belief]'),Character(')')]

        #onr report manual tree
        
        '''
        character_list = [Character('?'),Character('('),\
        	Character('?'),Character('('),\
        	Character('<!>'),Character('('),\
        	Character('(battery_low)'),Character(')'),Character('[resurface]'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('(wildlife_found)'),Character('?'),Character('('),\
        	Character('(in_comms)'),Character('[go_to_comms]'),Character(')'),Character('[report]'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('(mine_found)'),Character('?'),Character('('),\
        	Character('<!>'),Character('('),\
        	Character('(is_armed)'),Character(')'),Character('[disarm]'),Character(')'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('(benign_object_found)'),Character('[pick_up]'),Character('[take_to_drop_off]'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('?'),Character('('),\
        	Character('(likely_target_found)'),Character('[go_to_target]'),Character(')'),Character('random_walk'),Character(')'),\
        	Character(')')]
        '''
        '''
        character_list = [Character('?'),Character('('),\
        	Character('->'),Character('('),\
        	Character('(battery_low)'),Character('[resurface]'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('(wildlife_found)'),Character('?'),Character('('),\
        	Character('(in_comms)'),Character('[go_to_comms]'),Character(')'),Character('[report]'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('(mine_found)'),Character('?'),Character('('),\
        	Character('<!>'),Character('('),\
        	Character('(is_armed)'),Character(')'),Character('[disarm]'),Character(')'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('(benign_object_found)'),Character('[pick_up]'),Character('[take_to_drop_off]'),Character(')'),\
        	Character('->'),Character('('),\
        	Character('?'),Character('('),\
        	Character('(likely_target_found)'),Character('[go_to_target]'),Character(')'),\
        	Character('[random_walk]'),Character(')'),\
        	Character(')')]
        '''
        
        # updated multi-target manual tree
        character_list = [Character('?'),Character('('),\
            Character('->'),Character('('),\
            Character('(battery_low)'),Character('[resurface]'),Character(')'),\
            Character('->'),Character('('),\
            Character('(wildlife_found)'),Character('?'),Character('('),\
            Character('(in_comms)'),Character('[go_to_comms]'),Character(')'),Character('[report]'),Character(')'),\
            Character('->'),Character('('),\
            Character('(mine_found)'),Character('?'),Character('('),\
            Character('<!>'),Character('('),\
            Character('(is_armed)'),Character(')'),Character('[disarm]'),Character(')'),Character(')'),\
            Character('->'),Character('('),\
            Character('(benign_object_found)'),Character('[pick_up]'),Character(')'),\
            Character('->'),Character('('),\
            Character('(carrying_benign)'),Character('[take_to_drop_off]'),Character(')'),\
            Character('->'),Character('('),\
            Character('(likely_target_found)'),Character('[go_to_likely_target]'),Character(')'),\
            Character('[random_walk]'),\
            Character(')')]
        
        '''
        character_list = [Character('?'),Character('('),\
            Character('->'),Character('('),\
            Character('->'),Character('('),\
            Character('?'),Character('('),\
            Character('<!>'),Character('('),\
            Character('(likely_target_found)'),Character(')'),\
            Character('(is_armed)'), Character(')'),\
            Character('[random_walk]'),Character(')'),\
            Character('(carrying_benign)'),Character(')'),\
            Character('[pick_up]'),Character(')')]
        '''
        
        '''    
        character_list = [Character('?'),Character('('),\
            Character('?'),Character('('),\
            Character('->'),Character('('),\
            Character('(mine_found)'),Character('[disarm]'),Character(')'),\
            Character('[shortest_path]'),Character(')'),\
            Character('[go_to_comms]'),Character(')')]

        '''
        #cfg_word = Word(character_list)
        
        # cfg_word = createWord('? ( ? ( -> ( (mine_found) [disarm] ) [shortest_path] ) [go_to_comms] )')

        #cfg_word = createWord('?  (  ->  (  ?  (  <!>  (  (benign_object_found)  )  (benign_object_found)  <!>  (  (carrying_benign)  )  [take_to_drop_off]  )  [pick_up]  )  ->  (  (mine_found)  ?  (  (is_armed)  [disarm]  )  )  )')
        
        # Manual tree
        cfg_word_bla = createWord('? ( -> ( (target_found) ? ( (in_comms) [go_to_comms] ) [report] ) -> ( (mine_found) ? ( <!> ( (is_armed) ) [disarm] ) ) -> ( ? ( <!> ( (carrying_object) ) [take_to_drop_off] ) (object_found) [pick_up] ) -> ( (likely_target_found) [go_to_likely_target] ) -> ( [coverage] ) )')
        
        # Best performing final tree
        #cfg_word = createWord('? ( -> ( (is_armed) [disarm] ) -> ( <!> ( (likely_target_found) ) [go_to_likely_target] ) -> ( (benign_object_found) ? ( <!> ( (carrying_benign) ) [take_to_drop_off] ) ? ( [pick_up] ) ) -> ( (in_comms) ? ( <!> ( (at_surface) ) [report] ) ? ( [go_to_comms] ) ) -> ( ? ( <!> ( (at_surface) ) [report] ) (wildlife_found) [go_to_comms] ) )')
        #cfg_word = createWord('? ( -> ( (benign_object_found) ? ( [pick_up] ) ) -> ( (carrying_benign) <!> ( (benign_object_found) ) [take_to_drop_off] ) -> ( ? ( <!> ( (benign_object_found) ) ) (carrying_benign) ) -> ( [report] ? ( (wildlife_found) ) ? ( [go_to_comms] ) ) -> ( (is_armed) ? ( <!> ( (mine_found) ) [disarm] ) ) -> ( [coverage] ) )')
        #cfg_word = createWord('? ( -> ( (object_found) [pick_up] ) -> ( (target_found) <!> ( (in_comms) ) ? ( (at_surface) [report] ) [go_to_comms] ) -> ( (is_armed) [disarm] ) -> ( (object_found) ) -> ( (carrying_object) [take_to_drop_off] ) -> ( [go_to_likely_target] ) )')


	#######################################
	# Change cfg_word to show a different tree
	
	cfg_word = createWord('? ( ? ( (robot_visible) [go_to_child] ) -> ( (child_moving_away) [do_sounds] [move_toward] [do_bubbles] ) -> ( (child_moving_toward) [move_away] [do_any_reward_action] ) ) ')
	#######################################

	


        cfg_word.printWord()
        # cfg_word_filter = filterDuplicates(cfg_word)
        # cfg_word_filter.printWord()

        bt_root, bt = cfg_word.createBT()
        # bt_root, bt = cfg_word_filter.createBT()

        output_filename = 'manual_bt.tree'
        bt.write_config(output_filename) # Goes to home/scheidee/.ros

        includes = [True, True, True, True, True, True, True, True, True, True,\
            True, True, True, True, True, True, True, True, True, True, True,\
            True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True]

        new_word = exportBT(bt, includes)

        print("new_word:")
        new_word.printWord()
        print("subtrees:")
        subtrees = extract_subtrees(new_word)
        for subtree in subtrees:
            subtree.printWord()

        init_bt(bt)
        #bt.write_config('onr_example2.tree') # need to manually add in decorator nodes to config file/ implement it dur
        while not rospy.is_shutdown():   
        	tick_bt(bt)

    except rospy.ROSInterruptException: pass

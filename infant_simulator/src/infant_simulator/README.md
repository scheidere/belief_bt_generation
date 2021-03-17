# infantsim

TODOs

- Move infant position related functions to world/state for robot to access from there - DONE


ADD THESE FILES TO NEW SIM FOLDER - DONE
Overall files we need to merge or create:
bt_interface.py (should be done, maybe)
robot.py
infant.py
world.py
run_simulator.py (with old world.py)
scorer.py (need method of evaluating bt or bbt performance in sim episode)


robot.py:
	- state class:
		- update plot/plot update funcs to match ameer sim style - NOT DONE
		- update get position - NOT DONE
		- conditions dsi, si, sp - DONE
		- conditions move towards, move away, stationary - NOT DONE

	- robot class:
		- need to figure out if we need a loop in do iteration
		- condition_updates
		- set_action_status

	- robot controller
		- update for new domain - DONE
		- changed to controller (more general) - DONE
		- check if config is needed - DONE
		- controller needs to tell infant to maybe do something - DONE

infant.py:

	- give world infant info from its conditions

world.py:
	- get info from infant - DONE
	- write function to update bubble limit - DONE
	- keep track of current and previous distance between robot and infant (like from overhead cameras)

run_simulator.py:
	- use functions from infantsim world.py to update UnderwaterSimulator class (call it InfantSimulator???) - DONE
	- move generatereward functions to infantsimulator class
	- update test function to match infantsimulator


Ticking for Infant and Robot: Ryan's Edit
	- Both robot and infant should tick off "while not rospy__is__shutdown():"
	- For robot, while action is running, no new action is chosen
	- All actions have a duration: While an action is running, use a timer.
	- For infant, for every tick, there's a chance the infant looses interest and changes actions
		- Example: infant starts out simulator doing nothing (which is an action)
		- run a "infant changes mind" probability (trigger 1 every 3 seconds)
		- if changes mind
		- select a new action using our current method (using probability table based on robot actions)
		- then repeat tick
		- if doesn't changes mind
		- keep current action
		- then repeat tick

	- Function locations:
	- Infant and robot ticking should occur below "while not rospy__is__shutdown():" in controller class in robot.py
	- Infant actions should come from infant.py
	- robot actions should come from robot class in robot.py

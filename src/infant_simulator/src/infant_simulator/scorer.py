
# from world import World
import numpy as np
'''
Robot can submit "answers" or locations of targets to the scorer (i.e. basestation)
Scorer knows the correct answer

If robot gets answer correct, reward is -time it took (time meaning number of iterations (in robot controller))
If answer is wrong, scorer tells robot its "False" (so robot knows to go back and look again)

Robot should only be allowed to interact with the scorer within comm range and while at surface

If robot tries to communicate when not at surface or not within comm range, scorer returns "no response"

Need to extract how many iterations are taken at the "time" the robot reports an answer
'''


class Scorer():

    RESPONSE_CORRECT = 1
    RESPONSE_FALSE = 2
    RESPONSE_NONE = 3

    def __init__(self):
        # self.world = world
        self.distance = 0
        self.score = 0
        self.finished = False
        # self.max_iterations = max_iterations
        # self.belief_distance = self.world.config['environment_size'][0] + self.world.config['environment_size'][1]
        # self.detection_rewarded_tracker = np.zeros(self.world.num_nodes, dtype=bool)
        # self.action_rewarded_tracker = np.zeros(self.world.num_nodes, dtype=bool)

    def infant_sim_reward(self, infant_action, distance):
        # if condition child_moving_toward is true, we get positive reward cuz you did a good
        # otherwise, you stink and get nothing

        self.distance += distance

        if infant_action == True:
            self.score += 1
            return True
        else:
            return False

  
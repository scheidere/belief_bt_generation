#!/usr/bin/env python3

class Parameters:

    # Test Parameters
    stat_runs = 1  # Number of statistical runs to perform
    generations = 1000  # Number of generations for training NN
    motion_time_step = 1 # amount of time for agent motion instead of other actions
    time_step = 3  # Amount of time (seconds) agent moves each step
    agent_steps = 10
    run_graph_only = False  # Setting to true only runs the graphing function

    # Agent Parameters
    sensor_resolution = 360
    detection_radius = 3.5  # Meters
    agent_rad = 0.274  # Radius of turtlebot in m (used for size estimations and collision detection)
    max_vel = 0.2  # m/s
    max_rot_vel = 0.75  # rad / s
    buff = 0.01  # If the agent gets closer than this, consider it a collision
    bubble_cap = 10

    # infant parameters
    inf_gravity = 0.9 # if randomly higher than this value (so 10% of the time), then infant moves towards another toy instead
    inf_vel = 0.2 # m/s
    infant_rad = 0.25 # radius of child in m (for collision detection)

    # World Parameters
    x_dim = 6
    y_dim = 6
    coll_penalty = -100.0
    move_reward = 10.0
    fail_reward = -1.0
    n_objects = 3 # Number of toys in the environment


    agent_actions = {"move_toward": 0, "move_away": 1, "idle": 2, "spin": 3, "bubbles": 4,"lights": 5, "sounds": 6}
    infant_actions = {0:"Move towards",1:"Move away",2:"Move to object",3:"spin around",4:"stay still"}

    #agent_actions = {0:"Move away",1:"Move towards",2:"idle",3:"spin",4:"lights",5:"bubbles",6:"sound",7:"light_sound",8:"lights_bubbles",9:"sound_bubbles"}
    #infant_actions = {0:"Move towards",1:"Move away",2:"Move to object",3:"spin around",4:"stay still"}
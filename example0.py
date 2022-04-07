"""Load libraries"""
from enum import auto
from machine import automata
from machine import operations
from machine import dijkstra2 as dk
from machine import rob_callback as rc
from machine import map_generation as mpg
from machine import local_plant as local 
from machine import path_controller as pathc
from machine import robotarium_callbacks as rob
from machine import coordination_controller as coord
import numpy as np
import random



"""User inputs"""
#Size of the Matrix of states
W, H, N = 3, 2, 2

#Arena dimensions
WIDTH, HEIGHT = 3.2, 2

#Position tolerance
RADIUS = 0.06

#Communication parameters
COMMUNICATION_RADIUS = 2
PRIORITY_RADIUS = 1

#Simulation plotting parameters
SHOW_FIGURE = True
REAL_TIME = True


"""Simulation setup"""
#Creating the set of States and Events
(states, events) = mpg.generate_states_and_events(W, H)

#Creating the base for transitions
(transitions, dic_pos, matrix_states) = mpg.generate_transitions_and_positions(
    states, events, W, H, WIDTH, HEIGHT)
G = automata.Automaton(transitions, matrix_states[0][0])

#Creating the Disabled Events List
del_in = dict()
del_out = dict()
for i in range(N):
    del_in[i] = set()
    del_out[i] = set()

#Creating auxiliary variables
task_conclusion = [False]*N
transition_conclusion = [True]*N

#Loading initial state and a goal
initial_pos = [matrix_states[0][0], matrix_states[0][2]]
final_pos = [matrix_states[0][2], matrix_states[0][0]]
goal_points = np.ones([3, N])

#Loading initial real state and logical state
real_state = initial_pos
logical_state = initial_pos


"""Simulation"""
#Starting the simulation
local.goal_update(goal_points, dic_pos, real_state)
r = rob.Simulator(N, SHOW_FIGURE, goal_points, REAL_TIME)

#Simulation loop
while True:
    r.step_begin()

    #Updating the real state
    for i in range(N):
        real_state[i] = local.real_transition(real_state[i], logical_state[i], 
        dic_pos, r.x_poses, i, RADIUS)
        del_out[i].update(coord.generate_del(transitions, real_state[i], 
        logical_state[i]))

    #Checking for event conclusion
    agents = pathc.event_conclusion_single(real_state, logical_state, 
    final_pos)

    #Loop for agents tha have to calculate new paths
    for i in agents:
        
        #Updating the disabled events list for agents that have concluded 
        # events
        del_in = coord.receive_del(del_in, del_out, i, N, transitions,
        real_state, COMMUNICATION_RADIUS)

    #Check for total blocking
        total_block = pathc.total_momentary_blocking(real_state[i],
        del_in, transitions)

        if total_block:
            #Set real state as logical state
            logical_state[i] = real_state[i]
            
        else:
            #Try to calculate the path, if not possible a partial blocking is 
            # happeningclear
            partial_block, path, partial_possible = pathc.path(G, del_in[i], real_state[i], final_pos[i])

            """Terminar de escrever a parte sobre bloqueio parcial momentâneo"""
            if partial_block:
                logical_state = [real_state[i], random.sample(partial_possible,1)]
                print(logical_state)
                #Set a random accessible state as logical
                """Escrever uma função que calculca o Dijkstra e informa se o caminho foi possível ou não"""
            else:
                #Calculate path with Dijkstra
                pass

    local.goal_update(goal_points, dic_pos, logical_state)
    r.step_end(goal_points)

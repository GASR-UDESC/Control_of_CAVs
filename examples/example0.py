from machine import automata
from machine import operations
from machine import dijkstra2 as dk
from machine import rob_callback as rc
from machine import map_generation as mpg
from machine import local_plant as local 
from machine import path_controller as pathc
from machine import robotarium_callbacks as rob
import numpy as np

from machine.path_controller import event_conclusion

#Size of the Matrix of states
W, H, N = 3, 2, 2

#Arena dimensions
WIDTH, HEIGHT = 3.2, 2

#Position tolerance
RADIUS = 0.06

#Creating the set of States and Events
(states, events) = mpg.generate_states_and_events(W, H)

#Creating the automaton itself and its positions
(trans, dic_pos, matrix_states) = mpg.generate_transitions_and_positions(states, events, W, H, WIDTH, HEIGHT)
G = automata.Automaton(trans, events[0])

#Loading initial state and a goal
initial_pos = [matrix_states[0][0], matrix_states[0][2]]
final_pos = [matrix_states[0][2], matrix_states[0][0]]
goal_points = np.ones([3, N])

#Loading initial real state and logical state
real_state = initial_pos
logical_state = initial_pos

#Starting the simulation
SHOW_FIGURE = True
REAL_TIME = True
local.goal_update(goal_points, dic_pos, final_pos)
print(goal_points)
r = rob.Simulator(N, SHOW_FIGURE, goal_points, REAL_TIME)

#Simulation loop
while True:
    r.step_begin()
    (task_conclusion, event_conclusion) = pathc.event_conclusion(real_state, logical_state, final_pos)
    if task_conclusion:
        pass
    else:
        if event_conclusion:
            #calculate path
            pass
        else:
            #update simulation
            pass
    local.goal_update(goal_points, dic_pos, final_pos)
    r.step_end(goal_points)
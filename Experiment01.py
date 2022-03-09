
from machine import automata
from machine import operations
from machine import dijkstra2 as dk
from machine import rob_callback as rc
from machine import map_generation as mpg
from machine import local_plant as local 
from rps import robotarium
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import random
def EXPERIMENT01():
    #Size of the Matrix of states
    W, H, N = 3, 2, 2

    #Arena dimensions
    WIDTH, HEIGHT = 3.2, 2

    #Position tolerance
    RADIUS = 0.06
    
    #Creating the set of States and Events
    (states, events) = mpg.generate_states_and_events(W, H)

    #Creating the automaton itself and its positions
    (trans, DIC_POSITIONS, Matrix_states) = mpg.generate_transitions_and_positions(states, events, W, H, WIDTH, HEIGHT)
    G = automata.Automaton(trans, events[0])

    
    #Loading initial state and a goal
    initial_pos = [Matrix_states[0][0],Matrix_states[0][2]]
    Final_pos = [Matrix_states[0][2],Matrix_states[0][0]]
    goal_points = np.ones([3, N])

    real_state = initial_pos
    logical_state = initial_pos

    #Path planning variables
    T = [None]*N
    S = [None]*N
    T_optimal=[None]*N
    S_optimal=[None]*N

    priority_radius = [2]*N
    priority_radius[0] = 2



    communication_radius = [3]*N
    blacklist = dict()
    calculating = [True]*N
    defined_path = dict()
    calculating=[True]*N
    for i in range(N):
        blacklist[i] = []
        defined_path[i] = []

    #Control variables



    # Initializing the states list
    initial_points = rc.FC_SET_ALL_POSITIONS(DIC_POSITIONS, initial_pos)

    # Initializing the robotarium
    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_points, sim_in_real_time=True)
    single_integrator_position_controller = create_si_position_controller()
    __, uni_to_si_states = create_si_to_uni_mapping()
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()
    x = r.get_poses()
    x_si = uni_to_si_states(x)#"""
    r.step()
    

    # Creating an structure of past states during actual order
    past = dict()
    for s in range(N):
        past[s] = []
    while real_state != Final_pos:
        x = r.get_poses()
        x_si = uni_to_si_states(x)
        # Update Real State
        for i in range(N):
            blacklist[i] = []
            past_state[i] = real_state[i]
            real_state[i] = local.real_transition(real_state[i], logical_state[i],DIC_POSITIONS,x,i,RADIUS)
            if past_state[i] != real_state[i]:
                past_state2[i] = past_state[i]
        for i in range(N):
            if real_state[i] != Final_pos[i]:
                if real_state[i] == pivot_state[i]:
                    #Recalculus of route is necessary
                    calculating[i] = True
                elif real_state[i] == logical_state[i]:
                    #Update Robotarium state orders, no recalculus is needed
                    defined_path[i].pop(0)
                    logical_state[i] = defined_path[i][1]
                if calculating[i]:
                    for j in range(N):
                        if j != i:
                            d_real = dk.DIST(G, real_state[j], communication_radius[j])
                            if real_state[i] in d_real:
                                #Update blacklist[i]
                                if S[j] != None and len(defined_path[j]) > 1:

                                    start_black = True
                                    for k in range(len(defined_path[j])):
                                        if start_black:
                                            blacklist[i] = rc.add_black_real_logical(G, blacklist[i], defined_path[j][k], defined_path[j][k+1])
                                            start_black = False
                                        else:
                                            blacklist[i] = rc.add_black3(G, blacklist[i], defined_path[j][k])
                                            start_black = False
                                else:
                                    blacklist[i] = rc.add_black3(G, blacklist[i], real_state[j])

                    #Update Path[i]
                    (T_optimal[i], S_optimal[i]) = dk.PATH2(G, [], real_state[i], Final_pos[i])
                    try:
                        (T[i], S[i]) = dk.PATH2(G, blacklist[i], real_state[i], Final_pos[i])
                        if len(S[i]) > priority_radius[i]:

                            index = list(range(priority_radius[i]))
                        else:
                            index = list(range(len(S[i])))

                        defined_path[i] = list()
                        for j in index:
                            defined_path[i].append(S[i][j])
                        if len(S[i]) > len(S_optimal[i]):
                            if len(defined_path[i]) > 2:
                                for j in reversed(range(2, len(defined_path[i]))):
                                    if defined_path[i][j] not in S_optimal[i]:
                                        defined_path[i].pop(j)
                        pivot_state[i] = defined_path[i][-1]
                        logical_state[i] = S[i][1]
                        if logical_state[i] == past_state2[i]:

                            try:

                                blacklist[i] = rc.add_black3(G, blacklist[i], past_state2[i])

                                (T[i], S[i]) = dk.PATH2(G, blacklist[i], real_state[i], Final_pos[i])
                                if len(S[i]) > priority_radius[i]:

                                    index = list(range(priority_radius[i]))
                                else:
                                    index = list(range(len(S[i])))

                                defined_path[i] = list()
                                for j in index:
                                    defined_path[i].append(S[i][j])
                                pivot_state[i] = defined_path[i][-1]
                                logical_state[i] = S[i][1]
                            except:
                                pass
                    except:

                        blocked = rc.check_block(G.transitions, real_state[i], blacklist[i])
                        if blocked:
                            S[i] = [real_state[i]]
                            T[i] = [None]
                            defined_path[i] = [S[i][0]]
                            pivot_state[i] = S[i][0]
                            logical_state[i] = S[i][0]

                        else:
                            white_auto = G.transitions[real_state[i]]
                            white_keys = white_auto.keys()
                            white_list = list()
                            for j in white_keys:
                                # print(j)
                                if j not in blacklist[i]:
                                    if white_auto[j] != past_state2[i]:
                                        white_list.append(j)
                                    else:
                                        past_event = j


                            if len(white_list)>=1:
                                white_event = random.choice(white_list)
                            elif past_event not in blacklist[i]:
                                white_event = past_event
                            S[i] = [real_state[i], white_auto[white_event]]
                            T[i] = [white_event]

                            defined_path[i] = [real_state[i], white_auto[white_event]]

                            pivot_state[i] = S[i][1]
                            logical_state[i] = S[i][1]

                    calculating[i] = False

            else:
                #Reached final position
                # Reached final position
                logical_state[i] = real_state[i]


        for j in range(N):
            if logical_state[j] != None:
                rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, logical_state[j])
            else:
                rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, real_state[j])
        dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
        dxu = si_to_uni_dyn(dxi, x)
        r.set_velocities(np.arange(N), dxu)

        r.step()

    r.call_at_scripts_end()

    time_of_execution = r._iterations*0.033
    return(time_of_execution)

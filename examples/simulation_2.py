from machine import automata
from machine import operations
from machine import dijkstra2 as dk
from machine import rob_callback as rc
from rps import robotarium
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import random
import time

duration = 60   #duration in seconds
print('Beginning Simulation 2')
print('Random initial positions and goals')
print('Time for each experiment: ' + str(duration))
def base_experiment(duration, agents):
    timer_start = time.time()
    #Size of the Matrix of states
    w, h, N = 6, 6, agents

    #Creating States
    number_of_states = w*h
    states = [None]*number_of_states

    #Defining the positions of each State
    for i in range(number_of_states):
        states[i] = automata.State('S'+str(i))


    # Creating events
    number_of_events = 4*w*h+2*(w+h-2)
    events = [None]*number_of_events
    for i in range(number_of_events):
        events[i] = automata.Event(('e'+str(i)), 1, True)

    #Creating the automaton itself and its positions
    trans = dict()
    Matrix_states = [[0 for x in range(w)] for y in range(h)]
    #Data about positions
    wsize = 3.2
    hsize = 2
    Initial_point = np.array([0, 0, 0])
    wdif = wsize/(w)
    hdif = hsize/(h)
    positions = [[0 for x in range(w)] for y in range(h)]
    DIC_POSITIONS = dict()
    counter_states = 0
    for i in range(h):
        for j in range(w):
            Matrix_states[i][j] = states[counter_states]
            trans[states[counter_states]] = dict()
            positions[i][j] = [x + y for x, y in zip(Initial_point, [(j-(w-1)/2)*wdif, (-i+(h-1)/2)*hdif, 0])]
            DIC_POSITIONS[states[counter_states]] = positions[i][j]
            counter_states += 1
    counter_events = 0

    for i in range(h):
        for j in range(w):
            if i<(h-1):
                trans[Matrix_states[i][j]][events[counter_events]] = Matrix_states[i+1][j]
                counter_events += 1
            if i>(0):
                trans[Matrix_states[i][j]][events[counter_events]] = Matrix_states[i - 1][j]
                counter_events += 1
            if j<(w-1):
                trans[Matrix_states[i][j]][events[counter_events]] = Matrix_states[i][j+1]
                counter_events += 1
            if j>(0):
                trans[Matrix_states[i][j]][events[counter_events]] = Matrix_states[i][j-1]
                counter_events += 1

    G = automata.Automaton(trans, events[0])

    #Creating inputs for robotarium
    RADIUS = 0.06

    #Experiment 1 - Position swap

    non_repetition = list()
    Final_pos = [0]*N
    Initial_pos = [0]*N
    trans_counter = list()
    trans_buffer = [1]*N
    trans_logic = [True]*N
    trans_per_state = dict()
    tasks_per_state = dict()
    agent_shuffle = list(range(N))
    timeing = [True]*N
    wait_time = [0]*N
    start_time = [0]*N
    for i in states:
        trans_per_state[i] = 0
        tasks_per_state[i] = 0
    recalculus = 0
    for i in range(N):
        Initial_pos[i] = random.choice(list(set(states)-set(non_repetition)))
        Final_pos[i] = random.choice(list(set(states)-set([Initial_pos[i]])))
        non_repetition.append(Initial_pos[i])
    real_state = Initial_pos
    pivot_state = [[]]*N
    past_state = [[]]*N
    past_state2= [[]]*N

    #Path planning variables
    T = [None]*N
    S = [None]*N
    T_optimal=[None]*N
    S_optimal=[None]*N
    logical_state = [None] * N
    priority_radius = [2]*N



    communication_radius = [4]*N
    blacklist = dict()
    calculating = [True]*N
    defined_path = dict()
    calculating=[True]*N
    for i in range(N):
        blacklist[i] = []
        defined_path[i] = []

    #Control variables
    possible = rc.FC_POSSIBLE_STATES_ARRAY(DIC_POSITIONS)
    goal_points = np.ones([3, N])


    # Initializing the states list
    initial_points = rc.FC_SET_ALL_POSITIONS(DIC_POSITIONS, Initial_pos)

    # Initializing the robotarium
    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_points, sim_in_real_time=False)
    single_integrator_position_controller = create_si_position_controller()
    __, uni_to_si_states = create_si_to_uni_mapping()
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()
    # Plotting Parameters
    # Plotting Parameters
    CM = np.zeros([N,3])
    for i in range(N):
        for j in range(3):
            CM[i,j] = random.random()
    goal_marker_size_m = 0.2
    robot_marker_size_m = 0.15
    #"""
    marker_size_goal = determine_marker_size(r, goal_marker_size_m)
    marker_size_robot = determine_marker_size(r, robot_marker_size_m)
    font_size = determine_font_size(r, 0.1) #"""
    line_width = 5
    x = r.get_poses()
    x_si = uni_to_si_states(x)

    #"""
    #Read and show an image
    goal_marker_point=np.array([[None]*N, [None]*N])
    for i in range(N):
        goal_marker_point[0][i] = DIC_POSITIONS[Final_pos[i]][0]
        goal_marker_point[1][i] = DIC_POSITIONS[Final_pos[i]][1]
    goal_markers = [r.axes.scatter(np.array(goal_marker_point[0,ii]), np.array(goal_marker_point[1,ii]), s=0.7*marker_size_goal, marker='s', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width,zorder=-3)
    for ii in range(np.array(goal_points.shape[1]))]
    robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width)
    for ii in range(goal_points.shape[1])]#"""
    r.step()

    RUN = True
    first = [True]*N
    finished = [0]*N

    # Creating an structure of past states during actual order
    past = dict()
    for s in range(N):
        past[s] = []
    string_size = list()
    timer_duration = time.time()-timer_start
    while timer_duration < duration:
        #print(trans_buffer)
        #print('Running Experiment: ', (N-1))
        #print('Remaining time: ', (duration-timer_duration))
        x = r.get_poses()
        x_si = uni_to_si_states(x)
        # Update Real State
        for i in range(N):
            blacklist[i] = []
            past_state[i] = real_state[i]
            real_state[i] = rc.FC_MAKE_REAL_TRANSITION(possible, states, real_state[i], x, i, RADIUS)
            if past_state[i] != real_state[i]:
                past_state2[i] = past_state[i]
        random.shuffle(agent_shuffle)
        #print(agent_shuffle)
        for i in agent_shuffle:
            if real_state[i] != Final_pos[i]:
                if real_state[i] == pivot_state[i]:
                    #Recalculus of route is necessary
                    if timeing[i]:
                        wait_time[i] = 3*random.random()
                        start_time[i] = time.time()
                        timeing[i] = False
                    calculating[i] = True
                    recalculus += 1 
                    if trans_logic[i]:
                        trans_buffer[i] = trans_buffer[i] + 1
                        trans_logic[i] = False

                elif real_state[i] == logical_state[i]:
                    #Update Robotarium state orders, no recalculus is needed
                    defined_path[i].pop(0)
                    logical_state[i] = defined_path[i][1]
                    trans_buffer[i] = trans_buffer[i] + 1
            
                if calculating[i]:
                    actual_time = time.time()-start_time[i]
                    #print(wait_time[i],'   ', actual_time)
                    if actual_time>wait_time[i]:
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
                            trans_logic[i] = True
                        except:

                            blocked = rc.check_block(G.transitions, real_state[i], blacklist[i])
                            if blocked:
                                S[i] = [real_state[i]]
                                T[i] = [None]
                                defined_path[i] = [S[i][0]]
                                pivot_state[i] = S[i][0]
                                logical_state[i] = S[i][0]
                                trans_logic[i] = False

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
                                trans_logic[i] = True
                        timeing[i] = True
                        calculating[i] = False


            else:
                #Reached final position
                # Reached final position
                #goal_markers = [r.axes.scatter(np.array(goal_marker_point[0,ii]), np.array(goal_marker_point[1,ii]), s=marker_size_goal, marker='s', facecolors='none',edgecolors=[1,1,1],linewidth=line_width,zorder=-3)
                #for ii in range(np.array(goal_points.shape[1]))]
                Final_pos[i] = random.choice(list(set(states)-set([Final_pos[i]])))
                trans_counter.append(trans_buffer[i])
                #print('task completed', i, 'number of transitions:', trans_buffer[i])
                tasks_per_state[Final_pos[i]] += 1
                trans_per_state[Final_pos[i]] += trans_buffer[i]
                trans_buffer[i] = 0
                goal_markers = r.axes.scatter(np.array(goal_marker_point[0,i]), np.array(goal_marker_point[1,i]), s=0.7*marker_size_goal, marker='s', facecolors='None',color=[1,1,1],linewidth=line_width,zorder=-3)

                Final_pos[i] = random.choice(list(set(states)-set([real_state[i]])))
                goal_marker_point[0][i] = DIC_POSITIONS[Final_pos[i]][0]
                goal_marker_point[1][i] = DIC_POSITIONS[Final_pos[i]][1]
                goal_markers = r.axes.scatter(np.array(goal_marker_point[0,i]), np.array(goal_marker_point[1,i]), s=0.7*marker_size_goal, marker='s', color=CM[i,:], facecolors='None', linewidth=line_width,zorder=-3)


        for i in range(x.shape[1]):
            robot_markers[i].set_offsets(x[:2, i].T)
        # This updates the marker sizes if the figure window size is changed.
        # This should be removed when submitting to the Robotarium.
            robot_markers[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])

        for j in range(N):
            if logical_state[j] != None:
                rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, logical_state[j])
            else:
                rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, real_state[j])
        dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
        dxu = si_to_uni_dyn(dxi, x)
        r.set_velocities(np.arange(N), dxu)

        r.step()
        timer_duration = time.time()-timer_start
        
    r.call_at_scripts_end()
    time_of_execution = r._iterations*0.033
    return(trans_counter, recalculus, tasks_per_state, trans_per_state)


for a in range(35):
    base_experiment(duration, a+25),
    print('Simulations with' + str(a+1)+ ' agents')
from machine import automata
from machine import dijkstra2 as dk
from machine import rob_callback as rc
from rps import robotarium
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import random
import time

#Size of the Matrix of states
w, h, N = 8, 5, 20

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
whalf = wsize/2
hsize = 2.1
hhalf = hsize/2
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
RADIUS = 0.07
#"""
Initial_pos_array = np.concatenate((Matrix_states[0][:],Matrix_states[4][:]))
Initial_pos=Initial_pos_array.tolist()+[Matrix_states[2][0]]+[Matrix_states[2][3]]+ [Matrix_states[2][4]]+ [Matrix_states[2][7]]

Final_pos_array = np.concatenate((Matrix_states[4][:],Matrix_states[0][:]))
Final_pos = Final_pos_array.tolist()+[Matrix_states[2][3]]+[Matrix_states[2][0]]+[Matrix_states[2][7]]+[Matrix_states[2][4]]
#"""

# Choosing random initial and final positions

"""
Final_pos = [None]*N
Initial_pos = [None]*N
non_competition = list()
non_competition2 = list()

for i in range(N):
    Initial_pos[i] = random.choice(list(set(states)-set(non_competition)))
    non_competition.append(Initial_pos[i])
for i in range(N):
    Final_pos[i] = random.choice(list(set(states)-set(non_competition2)-set([Initial_pos[i]])))
    non_competition2.append(Final_pos[i])#"""

real_state = Initial_pos

#Path planning variables
N = len(Final_pos)
T = [None]*N
S = [None]*N
T_dj = [None]*N
S_dj = [None]*N
logical_state = [None] * N


#Control variables
possible = rc.FC_POSSIBLE_STATES_ARRAY(DIC_POSITIONS)
#possible_names = ['A', 'B', 'C', 'D', 'E', 'F']
goal_points = np.ones([3, N])



# Initializing the states list
initial_points = rc.FC_SET_ALL_POSITIONS(DIC_POSITIONS, Initial_pos)

# Initializing the robotarium
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_points, sim_in_real_time=True)
single_integrator_position_controller = create_si_position_controller()
__, uni_to_si_states = create_si_to_uni_mapping()
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# Plotting Parameters
CM = np.random.rand(N, 3)   #Random Colors
goal_marker_size_m = 0.2
robot_marker_size_m = 0.15
#"""
marker_size_goal = determine_marker_size(r, goal_marker_size_m)
marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r, 0.1)
#"""
line_width = 5
x = r.get_poses()
x_si = uni_to_si_states(x)

#"""
#Read and show an image
goal_marker_point=np.array([[None]*N, [None]*N])
for i in range(N):
    goal_marker_point[0][i] = DIC_POSITIONS[Final_pos[i]][0]
    goal_marker_point[1][i] = DIC_POSITIONS[Final_pos[i]][1]
goal_markers = [r.axes.scatter(np.array(goal_marker_point[0,ii]), np.array(goal_marker_point[1,ii]), s=marker_size_goal, marker='s', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width,zorder=-3)
for ii in range(np.array(goal_points.shape[1]))]
robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width)
for ii in range(goal_points.shape[1])]
#"""
r.step()

RUN = True
first = [True]*N
finished = [False]*N

# Creating an structure of past states during actual order
past = dict()
for s in range(N):
    past[s] = []
string_size = list()

# Creating an structure for counting time and verifying the path similarity to the optimal path
counting = [False]*N
t = [time.time()]*N
print(t)


while real_state != Final_pos:
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    for i in range(N):
        # Read real state
        real_state[i] = rc.FC_MAKE_REAL_TRANSITION(possible, states, real_state[i], x, i, RADIUS)
        # Update blacklist
        blacklist = list()
        for j in range(N):
            if logical_state[j] != None:
                blacklist = rc.add_black3(G, blacklist, logical_state[j])
                blacklist = rc.add_black_real_logical(G, blacklist, real_state[j], logical_state[j])
            else:
                blacklist = rc.add_black3(G, blacklist, real_state[j])
        # Check momentaneous block
        Block = rc.check_block(trans, real_state[i], blacklist)
        if Block:
            # If blocked, must stand still
            S[i] = [real_state[i]]
            T[i] = [None]
        else:
            # If not blocked, must search for a path
            (T_dj[i], S_dj[i]) = dk.PATH(G, [], real_state[i], Final_pos[i])
            (T[i], S[i]) = dk.PATH(G, blacklist, real_state[i], Final_pos[i])
            blacklist2 = blacklist
            if len(past[i]) > 1:
                blacklist2 = rc.add_black3(G, blacklist2, past[i][-1])
            Block2 = rc.check_block(trans, real_state[i], blacklist2)
            if not Block2:
                (T[i], S[i]) = dk.PATH(G, blacklist2, real_state[i], Final_pos[i])

        if real_state[i] == logical_state[i] or first[i]:
            if len(S[i]) > 1:
                first[i] = False
                past[i].append(real_state[i])
                logical_state[i] = S[i][1]
            else:
                logical_state[i] = S[i][0]
        if real_state[i] == Final_pos[i]:
            #print('Robo:', i)
            if not finished[i]:
                string_size.append(len(past[i]))
                finished[i] = True
            #past[i] = [real_state[i]]
            #Final_pos[i] = random.choice(list(set(states)-set([Final_pos[i]])))
            """
            goal_markers = [r.axes.scatter(np.array(goal_marker_point[0, ii]), np.array(goal_marker_point[1, ii]),
                                           s=marker_size_goal, marker='s', facecolors='none',
                                           edgecolors=np.array([[1, 1, 1], [1, 1, 1]]),
                                           linewidth=line_width, zorder=-3)
                            for ii in range(np.array(goal_points.shape[1]))]


            goal_marker_point[0][i] = DIC_POSITIONS[Final_pos[i]][0] + (0.01 * (i - N/2))
            goal_marker_point[1][i] = DIC_POSITIONS[Final_pos[i]][1] + (0.01 * (i - N/2))
            goal_markers = [r.axes.scatter(np.array(goal_marker_point[0, ii]), np.array(goal_marker_point[1, ii]),
                                           s=marker_size_goal, marker='s', facecolors='none', edgecolors=CM[ii, :],
                                           linewidth=line_width, zorder=-3)
                            for ii in range(np.array(goal_points.shape[1]))]
            #"""
            #finished[i] += 1
    for j in range(N):
        if logical_state[j] != None:
            rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, logical_state[j])
        else:
            rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, real_state[j])
    # Update the plot
    #"""
    for i in range(x.shape[1]):
        robot_markers[i].set_offsets(x[:2, i].T)
        # This updates the marker sizes if the figure window size is changed.
        # This should be removed when submitting to the Robotarium.
        robot_markers[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])
    #"""
    dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
    dxu = si_to_uni_dyn(dxi, x)
    r.set_velocities(np.arange(N), dxu)
    r.step()
    #print(len(string_size))

    # print(real_state, logical_state)
r.call_at_scripts_end()

media_interacoes = 0
tamanho= len(string_size)
for i in range(tamanho):
    media_interacoes += string_size[i]/tamanho
print(string_size)
print(media_interacoes)
#CRIAR
#blacklist local
#tempo de espera
print(r._iterations*0.033)
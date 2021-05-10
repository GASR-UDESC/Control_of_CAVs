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
Initial_pos = (Matrix_states[0][:]+Matrix_states[4][:])


Final_pos = (Matrix_states[4][:]+Matrix_states[0][:])

#"""

real_state = Initial_pos
pivot_state = [[]]*N

#Path planning variables
N = len(Final_pos)
T = [None]*N
S = [None]*N
T_dj = [None]*N
S_dj = [None]*N
logical_state = [None] * N
priority_radius = [2]*N



buffer = [0]*N
communication_radius = [2]*N
blacklist = dict()
blacklist_individual = dict()
calculating = [True]*N
defined_path = dict()
calculating=[True]*N
for i in range(N):
    blacklist[i] = []
    defined_path[i] = []
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



"""# Plotting Parameters
CM = np.random.rand(N, 3)   #Random Colors
goal_marker_size_m = 0.2
robot_marker_size_m = 0.15"""
#"""
gt_img = plt.imread('MAP.png')
x_img = np.linspace(-1.0, 1.0, gt_img.shape[1])
y_img = np.linspace(-1.0, 1.0, gt_img.shape[0])
gt_img_handle = r.axes.imshow(gt_img, extent=(positions[0][0][0], positions[-1][-1][0], positions[0][0][1], positions[-1][-1][1]))#"""
t1 = time.time()
t2 = 0
while t2 < 1:
    t2 = time.time()-t1
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    for j in range(N):
        rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, real_state[j])
    dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
    dxu = si_to_uni_dyn(dxi, x)
    r.set_velocities(np.arange(N), dxu)
    r.step()
"""
marker_size_goal = determine_marker_size(r, goal_marker_size_m)
marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r, 0.1) #"""
line_width = 5
x = r.get_poses()
x_si = uni_to_si_states(x)

"""
#Read and show an image
goal_marker_point=np.array([[None]*N, [None]*N])
for i in range(N):
    goal_marker_point[0][i] = DIC_POSITIONS[Final_pos[i]][0]
    goal_marker_point[1][i] = DIC_POSITIONS[Final_pos[i]][1]
goal_markers = [r.axes.scatter(np.array(goal_marker_point[0,ii]), np.array(goal_marker_point[1,ii]), s=marker_size_goal, marker='s', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width,zorder=-3)
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
while real_state != Final_pos:
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    # Update Real State
    for i in range(N):
        blacklist[i] = []
        real_state[i] = rc.FC_MAKE_REAL_TRANSITION(possible, states, real_state[i], x, i, RADIUS)


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
                            #(i,j)
                            #Update blacklist[i]
                            if S[j] != None and len(defined_path[j]) > 1:

                                start_black = True
                                #print(defined_path[j])
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
                try:
                    (T[i], S[i]) = dk.PATH2(G, blacklist[i], real_state[i], Final_pos[i])
                    if len(S[i])>priority_radius[i]:

                        index = list(range(priority_radius[i]))
                    else:
                        index = list(range(len(S[i])))

                    defined_path[i] = list()
                    for j in index:
                        defined_path[i].append(S[i][j])
                    pivot_state[i] = defined_path[i][-1]
                    logical_state[i] = S[i][1]

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
                        # print(i)
                        # print('black',blacklist)
                        for j in white_keys:
                            # print(j)
                            if j not in blacklist[i]:
                                white_list.append(j)
                        white_event = random.choice(white_list)
                        # print('white',white_list)
                        # print(i)
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

    """for i in range(x.shape[1]):
        robot_markers[i].set_offsets(x[:2, i].T)
        # This updates the marker sizes if the figure window size is changed.
        # This should be removed when submitting to the Robotarium.
        robot_markers[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])#"""

    for j in range(N):
        if logical_state[j] != None:
            rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, logical_state[j])
        else:
            rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, real_state[j])
    dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
    dxu = si_to_uni_dyn(dxi, x)
    r.set_velocities(np.arange(N), dxu)
    r.step()

#Waits one second before new instrucitions
t1 = time.time()
t2 = 0
while t2 < 1:
    t2 = time.time()-t1
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
    dxu = si_to_uni_dyn(dxi, x)
    r.set_velocities(np.arange(N), dxu)
    r.step()

Final_pos = (Matrix_states[0][:]+Matrix_states[4][:])

while real_state != Final_pos:
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    # Update Real State
    for i in range(N):
        blacklist[i] = []
        real_state[i] = rc.FC_MAKE_REAL_TRANSITION(possible, states, real_state[i], x, i, RADIUS)


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
                            #(i,j)
                            #Update blacklist[i]
                            if S[j] != None and len(defined_path[j]) > 1:

                                start_black = True
                                #print(defined_path[j])
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
                try:
                    (T[i], S[i]) = dk.PATH2(G, blacklist[i], real_state[i], Final_pos[i])
                    if len(S[i])>priority_radius[i]:

                        index = list(range(priority_radius[i]))
                    else:
                        index = list(range(len(S[i])))

                    defined_path[i] = list()
                    for j in index:
                        defined_path[i].append(S[i][j])
                    pivot_state[i] = defined_path[i][-1]
                    logical_state[i] = S[i][1]

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
                        # print(i)
                        # print('black',blacklist)
                        for j in white_keys:
                            # print(j)
                            if j not in blacklist[i]:
                                white_list.append(j)
                        white_event = random.choice(white_list)
                        # print('white',white_list)
                        # print(i)
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

    """for i in range(x.shape[1]):
        robot_markers[i].set_offsets(x[:2, i].T)
        # This updates the marker sizes if the figure window size is changed.
        # This should be removed when submitting to the Robotarium.
        robot_markers[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])#"""

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



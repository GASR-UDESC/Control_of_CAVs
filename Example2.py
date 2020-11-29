from machine import automata
from machine import operations
from machine import dijkstra2 as dk
from machine import rob_callback as rc
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import random

#Creating States
A = automata.State('A')
B = automata.State('B')
C = automata.State('C')
D = automata.State('D')
E = automata.State('E')
F = automata.State('F')

# Creating events
a = automata.Event('a', 1, True)
b = automata.Event('b', 1, True)
c = automata.Event('c', 1, True)
d = automata.Event('d', 1, True)
e = automata.Event('e', 1, True)
f = automata.Event('f', 1, True)
g = automata.Event('g', 1, True)
h = automata.Event('h', 1, True)
k = automata.Event('k', 1, True)
l = automata.Event('l', 1, True)
m = automata.Event('m', 1, True)
n = automata.Event('n', 1, True)
o = automata.Event('o', 1, True)
p = automata.Event('p', 1, True)

#Creating the automaton itself
trans = {A: {a: B, f: D}, B: {b: A, c: C, h: E}, C: {d: B, l: F}, D: {e: A, m: E}, E: {g: B, n: D, o: F}, F: {k: C, p: E}}
G = automata.Automaton(trans, A)

#Creating inputs for robotarium
scale = 1.5
A_point = [-0.8*scale, 0.4*scale, -np.pi/2]
B_point = [0*scale, 0.4*scale, 0*scale]
C_point = [0.8*scale, 0.4*scale, -np.pi/2]
D_point = [-0.8*scale, -0.4*scale, 0*scale]
E_point = [0*scale, -0.4*scale, 0*scale]
F_point = [0.8*scale, -0.4*scale, 0*scale]
DIC_POSITIONS = {'A': A_point, 'B': B_point, 'C': C_point, 'D': D_point, 'E': E_point, 'F': F_point}
Initial_pos = [A, C]
Final_pos = [C, B]
N = len(Final_pos)
RADIUS = 0.07

#Control variables
possible = rc.FC_POSSIBLE_STATES_ARRAY(DIC_POSITIONS)
possible_names = ['A', 'B', 'C', 'D', 'E', 'F']
possible_states = [A, B, C, D, E, F]
goal_points = np.ones([3, N])
real_state = Initial_pos
blacklist = list()
initial_points = rc.FC_SET_ALL_POSITIONS(DIC_POSITIONS, Initial_pos)

#Minimum path
T = [None]*N
S = [None]*N
logical_transition = [None] * N
logical_state = [None] * N
locked = [None]*N
buffer = [0]*N
first_iteration = [True]*N
first_iteration[1] = False

# Initializing the states list
initial_points = rc.FC_SET_ALL_POSITIONS(DIC_POSITIONS, Initial_pos)

# Initializing the robotarium
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_points, sim_in_real_time=True)
single_integrator_position_controller = create_si_position_controller()
__, uni_to_si_states = create_si_to_uni_mapping()
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

#Read and show an image
gt_img = plt.imread('map.png')
x_img = np.linspace(-1.0*scale, scale*1.0, gt_img.shape[1])
y_img = np.linspace(-1.0*scale, scale*1.0, gt_img.shape[0])
gt_img_handle = r.axes.imshow(gt_img, extent=(-0.87*scale, 0.95*scale, -0.52*scale, 0.52*scale))

#Read initial positions
x = r.get_poses()
x_si = uni_to_si_states(x)
r.step()

# Initialize the control
for i in range(N):
    (T[i], S[i]) = dk.PATH(G, real_state[i], Final_pos[i])
    logical_transition[i] = T[i][0]
    logical_state[i] = S[i][1]

#Calculates every frame, considering control restrictions and updates the plot
while real_state != Final_pos:
    x = r.get_poses()
    trans = {A: {a: B, f: D}, B: {b: A, c: C, h: E}, C: {d: B, l: F}, D: {e: A, m: E}, E: {g: B, n: D, o: F},
             F: {k: C, p: E}}
    trans2 = {A: {a: B, f: D}, B: {b: A, c: C, h: E}, C: {d: B, l: F}, D: {e: A, m: E}, E: {g: B, n: D, o: F},
             F: {k: C, p: E}}
    G = automata.Automaton(trans, A)
    G_whitelist = automata.Automaton(trans2, A)
    x_si = uni_to_si_states(x)
    blacklist = list()

    #Updates the blacklist
    for i in range(N):
        blacklist = rc.add_black3(G, blacklist, real_state[i])
        blacklist = rc.add_black3(G, blacklist, logical_state[i])
    for i in range(N):
        #Updates the real state
        real_state[i] = rc.FC_MAKE_REAL_TRANSITION(possible, possible_states, real_state[i], x, i, RADIUS)
        #Check if an event has been finished
        if real_state[i] == logical_state[i] or first_iteration[i]:
            #Check if the next event is blocked
            if logical_transition[i] in blacklist:
                for j in blacklist:
                    #Removes blocked events
                    G_whitelist.remove_transitions(j)
                try:
                    (T[i], S[i]) = dk.PATH(G_whitelist, real_state[i], Final_pos[i])
                except:
                    #If the goal point is occupied, checks if there is an momentaneous block
                    (momentaneous_blocked, new_goal) = rc.check_momentaneous_block(G_whitelist, real_state[i], blacklist)
                    if not momentaneous_blocked:
                        #Randomly chooses a state, if no path leads to the goal point
                        new_state = random.choice(new_goal)
                        (T[i], S[i]) = dk.PATH(G_whitelist, real_state[i], new_state)
                    else:
                        #If the agent is completely blocked, just maintain position
                        T[i] = []
                        S[i] = [real_state[i], real_state[i]]
            else:
                #If possible, just takes the fastest path
                (T[i], S[i]) = dk.PATH(G, real_state[i], Final_pos[i])
            try:
                logical_state[i] = S[i][1]
                logical_transition[i] = T[i][0]
            except:
                logical_state[i] = S[i][0]
            #Reupdates the blacklist after the new instructions by the control
            blacklist = rc.add_black3(G_whitelist, blacklist, real_state[i])
            blacklist = rc.add_black3(G_whitelist, blacklist, logical_state[i])
            if first_iteration:
                first_iteration[i] = False

    #Prints important details about the state of the system
    print('BEGINS FRAME')
    print('Events blocked (blacklist): ', blacklist)
    print('Real State', real_state)
    print('Logical State: ', logical_state)
    print('FRAME FINISHED')

    #Give the agents the commands
    for j in range(N):
        rc.FC_SET_GOAL_POINTS(DIC_POSITIONS, goal_points, j, logical_state[j].name)

    #Update the plot
    dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
    dxu = si_to_uni_dyn(dxi, x)
    r.set_velocities(np.arange(N), dxu)
    r.step()

#Finish the simulation after all agents reached their final positions
r.call_at_scripts_end()


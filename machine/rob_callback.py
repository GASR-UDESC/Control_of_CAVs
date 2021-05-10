from machine import automata
from machine import dijkstra2
import numpy as np

def FC_SET_GOAL_POINTS(dic_states, goal, n, where_to):
    """ Set the goal position to the required position"""
    goal_point = dic_states[where_to]
    goal[0][n] = goal_point[0]
    goal[1][n] = goal_point[1]
    goal[2][n] = goal_point[2]
    return goal

def FC_POSSIBLE_STATES_ARRAY(dic_states):
    """ Creates an array with the x and y information based on possible states"""
    possible_position = np.zeros([len(dic_states), 3])
    counter = 0
    for index in dic_states:
        possible_position[counter] = dic_states[index]
        counter += 1
    possible_position = np.delete(possible_position, 2, 1)
    return possible_position

def FC_SET_ALL_POSITIONS (dic_states, position):
    position_array = np.zeros([3, len(position)])
    for b in range(len(position)):
        position_array[0][b] = dic_states[position[b]][0]
        position_array[1][b] = dic_states[position[b]][1]
        position_array[2][b] = dic_states[position[b]][2]
    return(position_array)

def FC_MAKE_REAL_TRANSITION(possible_positions, possible_names, state_now, position_read, number_of_robot, safe_radius):
    L = len(possible_positions)
    distance_arrayx = np.zeros(L)
    distance_arrayy = np.zeros(L)
    distance_array = np.zeros([2, L])
    distance_points = np.zeros([L, 2])
    distance = np.zeros(L)
    for b in range(L):
        distance_arrayx[b] = position_read[0][int(number_of_robot)]-possible_positions[b][0]
        distance_arrayy[b] = position_read[1][int(number_of_robot)]-possible_positions[b][1]
    distance_array[0] = distance_arrayx
    distance_array[1] = distance_arrayy
    for c in range(L):
        for d in range(2):
            distance_points[c][d] = distance_array[d][c]
    state = state_now
    for b in range(L):
        distance[b] = np.linalg.norm(distance_points[b])
        if distance[b] < safe_radius:
            state = possible_names[b]

    return (state)

def add_black3(auto, blacklist, state):
    black = list()
    #print(state)
    k = auto.transitions.keys()
    for s in k:
        for e, s2 in auto.transitions[s].items():
            if s2 == state:
                #print(e, s2)
                black.append(e)
    #print(black)
    for block in black:
        if block not in blacklist:
            blacklist.append(block)
        #print(blacklist)
    return blacklist

def check_block(trans, real, blacklist):
    #print(trans[real].keys(),blacklist)
    Events = trans[real].keys()
    a = len(Events)
    b = list()
    for c in Events:
        if c in blacklist:
            b.append(c)#print('calcula')
    #print(b,len(b),a)
    if len(b) == a:
        return(True)
        print(b,blacklist)

    else:
        return(False)
    #print('NAO')
    return(True)

def add_black_real_logical(auto,blacklist, real,logical):
    black = list()
    for ev, j2 in auto.transitions[logical].items():
        if j2 == real:
            black.append(ev)
    for j3 in black:
        if j3 not in blacklist:
            blacklist.append(j3)
    return(blacklist)

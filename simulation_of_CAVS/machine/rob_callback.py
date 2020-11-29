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
        position_array[0][b] = dic_states[position[b].name][0]
        position_array[1][b] = dic_states[position[b].name][1]
        position_array[2][b] = dic_states[position[b].name][2]
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

def FC_MAKE_LOGICAL_TRANSITION(logical, n, dic_transitions, real, transitions_sequence, buffer):
    logical[n] = dic_transitions[logical[n]][transitions_sequence[n][int(buffer[n])]]
    buffer[n] += 1

def FC_MAKE_LOGICAL_TRANSITION(logical, n, dic_transition, path):
    logical[n] = dic_transition[path]

def logical_transition(input, n, path, buffer):
    input[n] = path[n][buffer[n]]
    buffer[n] += 1


def logical_states(auto, input_transition, input_state, real, n,buffer):
    input_state[n] = auto.transitions[real[n]][input_transition[n][buffer[n]]]
    buffer[n] += 1
    return (input_state)

def add_black(auto, blacklist, logic, N):
    for i in range(N):
        blocked_trans = auto.state_transition_set(logic[i])
        for j in range(len(blocked_trans)):
            single_transition_blocked = blocked_trans[j]
            if single_transition_blocked not in blacklist:
                blacklist.append(single_transition_blocked)
    return
def add_black2(auto, blacklist, state):
    black = list()
    for e, s2 in auto.transitions[state].items():
        black.append(e)
    for block in black:
        if block not in blacklist:
            blacklist.append(block)
    return

def add_black3(auto, blacklist, state):
    black = list()
    #print(state)
    k = auto.transitions.keys()
    for s in k:
        for e, s2 in auto.transitions[s].items():
            #print(s,s2)
            if s2 == state:
                #print(s2)
                black.append(e)
    #print(black)
    for block in black:
        if block not in blacklist:
            blacklist.append(block)
        #print(blacklist)
    return blacklist

def check_momentaneous_block(auto,s,blacklist):
    goal = list()
    for e, s2 in auto.transitions[s].items():
        if e not in blacklist:
            goal.append(s2)
    if len(goal) >= 1:
        blocked = False
    else:
        blocked = True
    #print(goal)
    return(blocked, goal)

# para iniciar, preciso:
# para cada robô, ler o estado inicial e colocar em initial_points, além disso calcular o menor caminho possível
# e guardar essa informação em uma lista para cada robô.

# para cada robô indicar a próxima transição e bloquear as transições levando ao estado lógico
# mas antes verificar se a transição está entre as proibidas, se estiver criar o automato sem
# as transições bloqueadas e calcular o dijkstra

#def checking_blocked_transitions

from machine import automata
import numpy as np

def generate_states_and_events(width_int, height_int):
    number_of_states = width_int*height_int
    set_of_states = [None]*number_of_states
    for i in range(number_of_states):
        set_of_states[i] = automata.State('S'+str(i))

    number_of_events = 4*number_of_states-2*(width_int+height_int)

    set_of_events = [None]*number_of_events
    for i in range(number_of_events):
        set_of_events[i] = automata.Event(('e'+str(i)), 1, True)

    return (set_of_states, set_of_events)


def generate_transitions_and_positions(set_of_states, set_of_events,
                                       width_int, height_int, width, height):
    transitions = dict()
    dic_positions = dict()
    matrix_of_states = [[0 for x in range(width_int)] for y in range(height_int)]
    
    initial_point = np.array([0, 0, 0])
    width_dif = width/width_int
    height_dif = height/height_int

    counter_states = 0
    for i in range(height_int):
        for j in range(width_int):
            matrix_of_states[i][j] = set_of_states[counter_states]
            transitions[set_of_states[counter_states]] = dict()
            position = [x + y for x, y in zip(initial_point, 
                        [(j-(width_int-1)/2)*width_dif,
                        (-i+(height_int-1)/2)*height_dif, 0])]
            dic_positions[set_of_states[counter_states]] = position
            counter_states += 1

    counter_events = 0
    for i in range(height_int):
        for j in range(width_int):
            if(i<(height_int-1)):
                event = set_of_events[counter_events]
                state = matrix_of_states[i+1][j]
                transitions[matrix_of_states[i][j]][event] = state
                counter_events += 1
            if(i>0):
                event = set_of_events[counter_events]
                state = matrix_of_states[i-1][j]
                transitions[matrix_of_states[i][j]][event] = state
                counter_events += 1
            if(j<(width_int-1)):
                event = set_of_events[counter_events]
                state = matrix_of_states[i][j+1]
                transitions[matrix_of_states[i][j]][event] = state
                counter_events += 1
            if(j>0):
                event = set_of_events[counter_events]
                state = matrix_of_states[i][j-1]
                transitions[matrix_of_states[i][j]][event] = state
                counter_events += 1            

    return (transitions, dic_positions, matrix_of_states)

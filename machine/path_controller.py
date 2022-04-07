from functools import partial
import dijkstar as dj
import copy
import random


def event_conclusion_single(real_states, logical_states, marked_states):
    """This function outputs a random list with the sequence of agents
    that have finished an event, but not its task. This will be useful
    for simulations which agents are supposed to finish only one task"""
    
    agents_list = list()

    for i in range(len(real_states)):
        task_concluded = False
        event_concluded = False

        if real_states[i] == marked_states[i] or real_states[i] in [
            marked_states[i]]:
            task_concluded = True
        elif real_states[i] == logical_states[i] or real_states[i] in [
            logical_states[i]]:
            event_concluded = True

        if not task_concluded and event_concluded:
            agents_list.append(i)
    
    random.shuffle(agents_list)

    return(agents_list)


def event_conclusion_multiple(real_states, logical_states, marked_states):
    """This function outputs a random list with the sequence of agents
    that have finished an event, but not its task. This will be useful
    for simulations which agents are supposed to finish multiple tasks"""

    agents_list = list()
    agents_finished_list = list()

    for i in range(len(real_states)):
        task_concluded = False
        event_concluded = False

        if real_states[i] == marked_states[i] or real_states[i] in [
            marked_states[i]]:
            task_concluded = True
        elif real_states[i] == logical_states[i] or real_states[i] in [
            logical_states[i]]:
            event_concluded = True

        if task_concluded:
            agents_finished_list.append(i)
    
        if event_concluded:
            agents_list.append(i)

    random.shuffle(agents_list)

    return(agents_list, agents_finished_list)


def total_momentary_blocking(real_state, disabled_events_list, transitions):
    
    possible_events = transitions[real_state].keys()

    set1 = set(possible_events)
    set2 = set(disabled_events_list)
    total_block = set1.issubset(set2)

    return (total_block)


def partial_momentary_blocking(real_state, marked_state, transitions):
    """Based on the function of state accessibility implemented in 
    https://github.com/GASR-UDESC/fitDES/blob/master/machine/operations.py"""

    states_to_be_visited = real_state.copy()
    visiting_state = states_to_be_visited.pop()

    for state in transitions.keys():
        state.visited = False
    
    while visiting_state:
        for state in transitions[visiting_state].values():

            if visiting_state == marked_state:
                return(False)

            if not state.visited:
                state.visited = True
                states_to_be_visited.add(state)
        
        try:
            visiting_state = states_to_be_visited.pop()
        except KeyError:
            visiting_state = None

    return(True)

def graph_generation(automaton, disabled_events, real_state):

    graph = dj.Graph()
    pos = automaton.transitions
    sts = automaton.states_set()
    real_state_out = set()

    for state in sts:
        t = pos[state].keys()
        for eve in t:
            if eve not in disabled_events:
                graph.add_edge(state, pos[state][eve], eve.weight)
            if state == real_state:
                real_state_out.add(pos[state][eve])

    return(graph,real_state_out)


def path(automaton, disabled_events, real_state, final_pos):

    graph, real_state_out = graph_generation(automaton, disabled_events, real_state)
    path_states = list()

    try:
        path_states = dj.find_path(graph, real_state, final_pos)
        partial_block = False
    
    except:
        partial_block = True


    return (partial_block, path_states, real_state_out)

"""def path(graph, real_state, marked_state):

    graph_path = dj.find_path(graph, real_state, marked_state)
    state_string = graph_path.nodes

    return(state_string)
"""

def partial_momentary_block(accessible_automaton, final_state):
    partial_block = True

    if final_state in accessible_automaton:
        partial_block = False

    return partial_block











"""Testing functions

def event_conclusion(real_state, logical_states, marked_states):
    task_concluded = False
    event_concluded = False

    if real_state == marked_states or real_state in [marked_states]:
        task_concluded = True
    if  real_state == logical_states or real_state in [logical_states]:
        event_concluded = True

    return (task_concluded, event_concluded)

"""
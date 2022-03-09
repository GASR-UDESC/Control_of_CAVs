import dijkstar as dj


def event_conclusion(real_state, logical_state, marked_states):
    task_concluded = False
    event_concluded = False

    if real_state in marked_states:
        task_concluded = True
    elif real_state in logical_state:
        event_concluded = True

    return (task_concluded, event_concluded)


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


def path(graph, real_state, marked_state):

    graph_path = dj.find_path(graph, real_state, marked_state)
    state_string = graph_path.nodes

    return(state_string)
from faulthandler import disable
import dijkstar as dj
from numpy import real

def generate_graph(transitions, disabled_events_list):
    
    graph = dj.Graph()
    states = transitions.keys()

    for state in states:
        events = transitions[state].keys()
        for event in events:
            if event not in disabled_events_list:
                graph.add_edge(state, event, event.weight)
                
    return graph


#def generate_priority_radius_transitions()


def generate_del(transitions, real_state, logical_states):

    disabled_events_list = set()
    states = transitions.keys()
    
    for state_in in states:
        events = transitions[state_in].keys()

        for event in events:
            state_out = transitions[state_in][event]

            if state_out == logical_states or state_out in [logical_states]:
                disabled_events_list.add(event)

            if state_in == logical_states or state_out in [logical_states]:
                if state_out == real_state:
                    disabled_events_list.add(event)


    return(disabled_events_list)


def receive_communication(transitions, real_state, communication_radius):
    
    initial = real_state
    distance = 0
    checking = dict()
    checking[distance] = {initial}
    communicable_states = set()

    while distance < communication_radius:

        checking[distance+1] = set()
        states_in = set(checking[distance])

        for state in states_in:
            events = set(transitions[state].keys())

            for event in events:
                state_out = transitions[state][event]
                checking[distance+1].add(state_out)
                communicable_states.add(state_out)
        
        distance += 1
    communicable_states.remove(initial)

    return(communicable_states)

def receive_del(del_in, del_out, agent_index, number_of_agents,
 transitions, real_states, communication_radius):

    del_in[agent_index].clear()

    communicable_states = receive_communication(transitions, 
    real_states[agent_index], communication_radius)
    
    for j in range(number_of_agents):
        if real_states[j] in communicable_states:
            del_in[agent_index].update(del_out[j])
            
    return del_in
import tracemalloc
import dijkstar as dj

def generate_graph(transitions, disabled_events_list):
    graph = dj.Graph()
    states = transitions.keys()

    for state in states:
        events = transitions[state].keys()
        for event in events:
            if event not in disabled_events_list:
                graph.add_edge(state, event, event.weight)
                
    return graph

import dijkstar as dj
from machine import automata
def graph_G(auto):
    graph = dj.Graph()
    pos = auto.transitions
    sts = auto.states_set()
    for state in sts:
        t = pos[state].keys()
        for eve in t:
            graph.add_edge(state, pos[state][eve], eve.weight)
    return graph
def path_trans(auto, path):
    sequence = [None] * (len(path) - 1)
    for s in range(len(path)-1):
        state = path[s]
        s_dic = auto.transitions[state]
        for trans, final in s_dic.items():
            if final == path[s+1]:
                sequence[s] = trans
    return(sequence)

def PATH(AUTO, INITIAL, FINAL):
    GRAPH = graph_G(AUTO)
    P_calculus = dj.find_path(GRAPH, INITIAL, FINAL)
    P_nodes = P_calculus.nodes
    TRANS = path_trans(AUTO, P_nodes)
    return(TRANS, P_nodes)

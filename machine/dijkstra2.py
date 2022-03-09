import dijkstar as dj

def path_trans(auto, path):
    sequence = [None] * (len(path) - 1)
    for s in range(len(path)-1):
        state = path[s]
        s_dic = auto.transitions[state]
        for trans, final in s_dic.items():
            if final == path[s+1]:
                sequence[s] = trans
    return(sequence)

def graph_G2(auto,black):
    graph = dj.Graph()
    pos = auto.transitions
    sts = auto.states_set()
    for state in sts:
        t = pos[state].keys()
        for eve in t:
            if eve not in black:
                graph.add_edge(state, pos[state][eve], eve.weight)
    return graph

def PATH2(AUTO, BLACK, INITIAL, FINAL):
    ###########################
    GRAPH = graph_G2(AUTO, BLACK)
    P_calculus = dj.find_path(GRAPH, INITIAL, FINAL)
    P_nodes = P_calculus.nodes
    TRANS = path_trans(AUTO, P_nodes)
    return(TRANS, P_nodes)

def DIST(AUTO, INITIAL, RADIUS):
    ###########################
    l = 0
    s = [INITIAL]
    dist = dict()
    dist[l] = s
    pos = AUTO.transitions
    checked = list()
    while l <= RADIUS:
        dist[l+1]=list()
        for s in dist[l]:
            for eve in pos[s].keys():
                s2 = pos[s][eve]
                for L in range(l+1):
                    for ss in dist[L]:
                        if ss not in checked:
                            checked.append(ss)
                if s2 not in checked:
                    dist[l+1].append(pos[s][eve])
                    checked.append(s2)
        l += 1
    dist.pop(l)
    s_list = list()
    for L in range(l):
        for ss in dist[L]:
            s_list.append(ss)
    return(s_list)


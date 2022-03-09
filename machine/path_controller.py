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


def partial_momentary_blocking(real_state, marked_state, disabled_events_list,
                               transitions):
    
    
    return(partial_block)
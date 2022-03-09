from cmath import sqrt


def set_goals(goals, agent, dic_positions, logical_state):
    goal_coord = dic_positions[logical_state]
    goals[0][agent] = goal_coord[0]
    goals[1][agent] = goal_coord[1]
    goals[2][agent] = goal_coord[2]

    return


def real_transition(real_state, logical_state, dic_positions, real_coordinates, agent, radius):
    real_x = real_coordinates[0][agent]
    real_y = real_coordinates[1][agent]
    logical_x = dic_positions[logical_state][0]
    logical_y = dic_positions[logical_state][1]
    
    distance = sqrt((real_x-logical_x)**2+(real_y-logical_y)**2).real
    if distance < radius:
        print(distance, radius)
        return logical_state
    else:
        return real_state




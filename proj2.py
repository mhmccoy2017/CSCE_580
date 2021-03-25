from environments.connect_four import ConnectFourState, ConnectFour


def heuistic(state: ConnectFourState) -> float:
    max_three_piece = 0
    min_three_piece = 0
    three = 0
    for player in [1, -1]:
    #using modified connect_four.py terminal state code to identify 3 consectuive  
    #Side note whoever wrote connect-four.py has HORT and VERT labeled backwards
        #VERT
        for pos_i in range(state.grid.shape[0]):
            num_in_row: int = 0
            for pos_j in range(state.grid.shape[1]):
                #checking min
                if state.grid[pos_i, pos_j] == player:
                    num_in_row += 1
                else:
                    num_in_row = 0
                if num_in_row == 3:
                    #Player addition check 
                    if player == -1:
                        min_three_piece += 1 
                    else: 
                        max_three_piece += 1
        #HORT 
        for pos_j in range(state.grid.shape[1]):
                num_in_row: int = 0
                for pos_i in range(state.grid.shape[0]):
                    if state.grid[pos_i, pos_j] == player:
                        num_in_row += 1
                    else:
                        num_in_row = 0

                    if num_in_row == 3:
                        #Checking which player got the 3 in the row
                        if player == -1:
                            min_three_piece += 1
                        else: 
                            max_three_piece += 1

        #DIAG
        for pos_i in range(state.grid.shape[0]):
                for pos_j in range(state.grid.shape[1]):
                    diag_incr: int = 0
                    num_in_row: int = 0
                    while (pos_i + diag_incr < state.grid.shape[0]) and (pos_j + diag_incr < state.grid.shape[1]):
                        if state.grid[pos_i + diag_incr, pos_j + diag_incr] == player:
                            num_in_row += 1
                        else:
                            num_in_row = 0

                        if num_in_row == 3:
                            if player == -1:
                                min_three_piece += 1
                            else:
                                max_three_piece += 1

                        diag_incr += 1

                    diag_incr: int = 0
                    num_in_row: int = 0
                    while (pos_i + diag_incr < state.grid.shape[0]) and (pos_j - diag_incr >= 0):
                        if state.grid[pos_i + diag_incr, pos_j - diag_incr] == player:
                            num_in_row += 1
                        else:
                            num_in_row = 0

                        if num_in_row == 3:
                            if player == -1:
                                min_three_piece += 1
                            else:
                                max_three_piece += 1

                        diag_incr += 1
    #print("max: ", max_three_piece)
    #print("min: ", min_three_piece)
    return max_three_piece - min_three_piece


def heuristic_minimax_search(state: ConnectFourState, env: ConnectFour, depth: int) -> int:
    """
    Returns the action to take
    :param state: connect four game state
    :param env: connect four environment
    :param depth: maximum search depth
    :return: action to take
    """
    value,move,d = max_value(state, env, depth) 
    return move

def max_value (state: ConnectFourState, env: ConnectFour, depth: int):
    move = None
    d = 0
    v = -1 * 1e6
    if env.is_terminal(state):
        return (env.utility(state), move, d)
    if depth < 0:
        for a in env.get_actions(state):
            v2 = heuistic(env.next_state(state,a))
        if v2 > v:
            move = a 
            v = v2
        return v2, move, d
    for a in env.get_actions(state):
        v2,a2, d = min_value(env.next_state(state,a), env ,depth-1)
        if v2 > v:
            v, move = v2, a
    return v, move, d
    """
    move = None
    d = 0
    v = -100000
    if env.is_terminal(state):
        return (env.utility(state), move)
    if depth <= 0:
        return (v,move)
    for a in env.get_actions(state):
        v2 = heuistic(a)
        if v2 > v
            move = a
            v = v2
    return v2, move
    """

def min_value (state: ConnectFourState, env: ConnectFour, depth: int):
    """
    move = None
    d = 0
    v = 100000
    if env.is_terminal(state):
        return (env.utility(state), move)
    if depth <= 0:
        return (v,move)
    for a in env.get_actions(state):
        v2 = heuistic(a)
        if v2 < v
            move = a
            v = v2
    return v2, move
    """
    move = None
    d = 0
    v = 1 * 1e6
    if env.is_terminal(state):
        return (env.utility(state), move, d)
    if depth < 0:
        for a in env.get_actions(state):
            v2 = heuistic(env.next_state(state,a))
        if v2 < v:
            move = a 
            v = v2
        return v2, move, d
    for a in env.get_actions(state):
        v2,a2, d = max_value(env.next_state(state,a), env, depth-1)
        if v2 < v:
            v, move = v2, a
    return v, move, d


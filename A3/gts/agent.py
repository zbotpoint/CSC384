"""
An AI player for Othello. 
"""

import operator
import random
import sys
import time

from numpy import empty

# You can use the functions in othello_shared to write your AI
from othello_shared import find_lines, get_possible_moves, get_score, play_move

states = dict()

def eprint(*args, **kwargs): #you can use this for debugging, as it will print to sterr and not stdout
    print(*args, file=sys.stderr, **kwargs)
    
# Method to compute utility value of terminal state
def compute_utility(board, color):
    black,white = get_score(board)
    if color == 1: # black
        return black - white
    elif color == 2: # white
        return white - black
    return 0 

# Better heuristic value of board
def compute_heuristic(board, color): #not implemented, optional
    black,white = get_score(board)
    if color == 1: # black
        score = black - white
    elif color == 2: # white
        score = white - black
    score += sum([1 for i in board[0] if i == color])
    + sum([-1 for i in board[0] if i == othercolor(color)])
    + sum([1 for i in board[-1] if i == color])
    + sum([-1 for i in board[-1] if i == othercolor(color)])
    + sum([1 for i in [row[0] for row in board] if i == color])
    + sum([-1 for i in [row[0] for row in board] if i == othercolor(color)])
    + sum([1 for i in [row[-1] for row in board] if i == color])
    + sum([-1 for i in [row[-1] for row in board] if i == othercolor(color)])
    
    return score
    
            

############ MINIMAX ###############################
def othercolor(color):
    if color == 1:
        return 2
    else:
        return 1        

def minimax_min_node(board, color, limit, caching = 0):
    enemy = othercolor(color)
    possible_moves = get_possible_moves(board,enemy)
    if not possible_moves: # terminal state
        return (None,compute_utility(board,color))

    if limit == 0:
        return (None, compute_heuristic(board, color))
        
    if (str(board),color) in states and caching==1:
        return states[(str(board),color)]

    scores = [
        (
            minimax_max_node(
                play_move(board, enemy, move[0], move[1]), color, limit-1, caching)[1],
            index
        )
        for index,move
        in enumerate(possible_moves)
    ]
    _score,min_move_index = min(scores,key=operator.itemgetter(0))
    
    states[(str(board),color)] = (possible_moves[min_move_index],_score)
    return (possible_moves[min_move_index],_score)

def minimax_max_node(board, color, limit, caching = 0): #returns highest possible utility
    possible_moves = get_possible_moves(board,color)
    if not possible_moves: # terminal state
        return (None,compute_utility(board,color))

    if limit == 0:
        return (None, compute_heuristic(board, color))

    if (str(board),color) in states and caching==1:
        return states[(str(board),color)]

    scores = [
        (
            minimax_min_node(
                play_move(board, color, move[0], move[1]), color, limit-1, caching)[1],
            index
        )
        for index,move
        in enumerate(possible_moves)
    ]
    _score, max_move_index = max(scores,key=operator.itemgetter(0))

    states[str(board),color] = (possible_moves[max_move_index],_score)
    return (possible_moves[max_move_index],_score)

def select_move_minimax(board, color, limit, caching = 0):
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    """
    move, score = minimax_max_node(board, color, limit, caching)
    return move

############ ALPHA-BETA PRUNING #####################
def alphabeta_min_node(board, color, alpha, beta, limit, caching=0, ordering=0):
    successors = [(move,play_move(board,othercolor(color),move[0],move[1])) for move in get_possible_moves(board, othercolor(color))]
    
    if not successors:
        return (None, compute_utility(board,color))
    
    if limit == 0:
        return (None, compute_utility(board, color))

    if (str(board),color) in states and caching==1:
        return states[(str(board),color)]
    
    if ordering == 1:
        successors.sort(key=lambda x: compute_utility(x[1],color))
    
    best_move = None
    val = 9999999
    for successor in successors:
        _next_move, next_val = alphabeta_max_node(successor[1], color, alpha, beta, limit - 1, caching, ordering)
        if next_val < val:
            best_move = successor[0]
            val = next_val

        beta = min(beta, next_val)
        if beta <= alpha:
            break
    states[str(board),color] = (best_move,val)
    return best_move, val


def alphabeta_max_node(board, color, alpha, beta, limit, caching=0, ordering=0):
    successors = [(move,play_move(board,color,move[0],move[1])) for move in get_possible_moves(board, color)]
    
    if not successors:
        return (None, compute_utility(board,color))
    
    if limit == 0:
        return (None, compute_utility(board, color))

    if (str(board),color) in states and caching==1:
        return states[(str(board),color)]

    if ordering == 1:
        successors.sort(key=lambda x: compute_utility(x[1],color),reverse=True)

    best_move = None
    val = -9999999
    for successor in successors:
        _next_move, next_val = alphabeta_min_node(successor[1], color, alpha, beta, limit - 1, caching, ordering)
        if next_val > val:
            best_move = successor[0]
            val = next_val

        alpha = max(alpha, next_val)
        if beta <= alpha:
            break
    states[str(board),color] = (best_move,val)
    return best_move, val


def select_move_alphabeta(board, color, limit, caching = 0, ordering = 0):
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    If ordering is ON (i.e. 1), use node ordering to expedite pruning and reduce the number of state evaluations. 
    If ordering is OFF (i.e. 0), do NOT use node ordering to expedite pruning and reduce the number of state evaluations. 
    """
    return (alphabeta_max_node(board, color, -9999999, 9999999, limit, caching, ordering))[0]

####################################################
def run_ai():
    """
    This function establishes communication with the game manager.
    It first introduces itself and receives its color.
    Then it repeatedly receives the current score and current board state
    until the game is over.
    """
    print("Othello AI") # First line is the name of this AI
    arguments = input().split(",")
    
    color = int(arguments[0]) #Player color: 1 for dark (goes first), 2 for light. 
    limit = int(arguments[1]) #Depth limit
    minimax = int(arguments[2]) #Minimax or alpha beta
    caching = int(arguments[3]) #Caching 
    ordering = int(arguments[4]) #Node-ordering (for alpha-beta only)

    if (minimax == 1): eprint("Running MINIMAX")
    else: eprint("Running ALPHA-BETA")

    if (caching == 1): eprint("State Caching is ON")
    else: eprint("State Caching is OFF")

    if (ordering == 1): eprint("Node Ordering is ON")
    else: eprint("Node Ordering is OFF")

    if (limit == -1): eprint("Depth Limit is OFF")
    else: eprint("Depth Limit is ", limit)

    if (minimax == 1 and ordering == 1): eprint("Node Ordering should have no impact on Minimax")

    while True: # This is the main loop
        # Read in the current game status, for example:
        # "SCORE 2 2" or "FINAL 33 31" if the game is over.
        # The first number is the score for player 1 (dark), the second for player 2 (light)
        next_input = input()
        status, dark_score_s, light_score_s = next_input.strip().split()
        dark_score = int(dark_score_s)
        light_score = int(light_score_s)

        if status == "FINAL": # Game is over.
            print
        else:
            board = eval(input()) # Read in the input and turn it into a Python
                                  # object. The format is a list of rows. The
                                  # squares in each row are represented by
                                  # 0 : empty square
                                  # 1 : dark disk (player 1)
                                  # 2 : light disk (player 2)

            # Select the move and send it to the manager
            if (minimax == 1): #run this if the minimax flag is given
                movei, movej = select_move_minimax(board, color, limit, caching)
            else: #else run alphabeta
                movei, movej = select_move_alphabeta(board, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()

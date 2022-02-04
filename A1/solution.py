#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
# for Sokoban specific classes and problems
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS

# SOKOBAN HEURISTICS


def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    # most of the work here is done in my evaluate_box function
    # this iterates over all the boxes, and evaluates each one
    # it returns infinity for unsolvable states
    # otherwise it returns the manhattan distance to the closest storage
    # sum it all up and return it as the heuristic

    # very simple, will improve if I have time, mostly focused on 
    # "punishing" bad states, i.e. unsolvable ones right now
    return \
        sum([
            evaluate_box(box, state)
            for box
            in state.boxes
        ])


def evaluate_box(box, state):
    # evaluates a box in a state
    # returns a very large number if the box is deadlocked, otherwise
    # returns the manhattan distance to the closest storage location

    # return 0 if we're at a store already
    if box in state.storage:
        return 0

    # return a very large number if we're cornered
    if (((box[0]-1, box[1]) in state.obstacles)
            or ((box[0]+1, box[1]) in state.obstacles)
            or (box[0] == 0 or box[0] == state.width-1)) \
        and (((box[0], box[1]-1) in state.obstacles)
             or ((box[0], box[1]+1) in state.obstacles)
             or (box[1] == 0 or box[1] == state.width-1)):
        return float('inf')

    # return a very large number if we're against a wall and no stores are against that wall
    if (box[0] == 0 and 0 not in [store[0] for store in state.storage]) \
            or (box[1] == 0 and 0 not in [store[1] for store in state.storage]) \
            or (box[0] == state.width-1 and state.width-1 not in [store[0] for store in state.storage]) \
            or (box[1] == state.height-1 and state.height-1 not in [store[1] for store in state.storage]):
        return float('inf')

    # otherwise return the manhattan distance to the closest store
    return min([
        mh_dist(box, store)
        for store
        in state.storage
    ])


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def heur_manhattan_distance(state):
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    total = \
        sum([
            min([
                mh_dist(box, store)
                for store
                in state.storage
            ])
            for box
            in state.boxes
        ])

    return total


def mh_dist(box, store):
    # finds the distance between a box and a store using manhattan distance
    return abs(box[0]-store[0]) + abs(box[1]-store[1])


def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight*sN.hval

# SEARCH ALGORITHMS


def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    se = SearchEngine('custom')

    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state,sokoban_goal_state,heur_fn,wrapped_fval_function)


    return se.search(timebound)


# uses f(n), see how autograder initializes a search line 88
def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    return None, None  # CHANGE THIS


def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    return None, None  # CHANGE THIS

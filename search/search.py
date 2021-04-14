# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def expand(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (child,
        action, stepCost), where 'child' is a child to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that child.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
          state: Search state

        For a given state, this should return a list of possible actions.
        """
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        """
          state: Search state
          action: action taken at state.
          next_state: next Search state after taking action.

        For a given state, this should return the cost of the (s, a, s') transition.
        """
        util.raiseNotDefined()

    def getNextState(self, state, action):
        """
          state: Search state
          action: action taken at state

        For a given state, this should return the next state after taking action from state.
        """
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    mystack = util.Stack()
    startNode = (problem.getStartState(), '', 0, [])
    mystack.push(startNode)
    visited = set()
    while mystack :
        node = mystack.pop()
        state, action, cost, path = node
        if state not in visited :
            visited.add(state)
            if problem.isGoalState(state) :
                path = path + [(state, action)]
                break
            succNodes = problem.expand(state)
            for succNode in succNodes:
                succState, succAction, succCost = succNode
                newNode = (succState, succAction, cost + succCost, path + [(state, action)])
                mystack.push(newNode)
    actions = [action[1] for action in path]
    del actions[0]
    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    #COMP90054 Task 1, Implement your A Star search algorithm here
    """Search the node that has the lowest combined cost and heuristic first.""" 
    "*** YOUR CODE HERE ***"
    #for search node
    open_node_list = util.PriorityQueue()
    #create search startNode (state,action,cost,pathToNode)
    #initial path should be (startState, '')
    startNode = (problem.getStartState(), '', 0, [(problem.getStartState(),'')])
    #push startNode and fCost
    open_node_list.push(startNode, 0)
    #for state node   
    closed_state_list = set()
    
    best_g = 999999 #initialise as infinity

    while not open_node_list.isEmpty():
        currentNode = open_node_list.pop()
        #get current node attributes
        state, act, cost, path = currentNode

        if state not in closed_state_list or cost < best_g:
            closed_state_list.add(state) #close state node
            best_g = cost #update best_g for reopening mechanism
            #if current node is goal state, return actions
            if problem.isGoalState(state):
                actions = [x[1] for x in path]
                del actions[0] #remove the empty first entry
                return actions
            #create successor search nodes
            succNodes = problem.expand(state)
            for succNode in succNodes:
                succState, succAction, succCost = succNode
                if succState not in closed_state_list:
                    #create new search node by state successors
                    newNode = ( succState, succAction, cost+succCost, path+[(succState, succAction)])
                    fCost = cost + succCost + heuristic(succState, problem)
                    open_node_list.push(newNode, fCost) #push into queue with priority of f
    return None

def recursivebfs(problem, heuristic=nullHeuristic) :
    #COMP90054 Task 2, Implement your Recursive Best First Search algorithm here
    "*** YOUR CODE HERE ***"
    ####a search node is a list consists of: state, action, g, path, f####
    startNode = [ problem.getStartState(), '', 0, [(problem.getStartState(),'')], 0+heuristic(problem.getStartState(),problem) ]
    # get recursive funtion return values: action-list and f-cost
    actions, f = rbfs(problem, startNode, float('inf'), heuristic)
    return actions
    
# Recursive function for ReBFS()
def rbfs(problem, node, f_limit, heuristic): 
    #get node attributes through list indices
    state = node[0]
    g = node[2] 
    path = node[3]
    f = node[4]
    #return solution if the node is goal state
    if problem.isGoalState(state):
        actions = [action[1] for action in path]
        del actions[0]
        return actions, None
    #create successors-node list
    successors = []
    #expand state nodes and create successor nodes(search nodes)
    succNodes = problem.expand(state)
    for succNode in succNodes:
        succState, succAction, succCost = succNode
        newNode = [ succState, succAction, g+succCost, path+[(succState,succAction)], g+succCost+heuristic(succState,problem) ]
        successors.append(newNode) #append new search node
    #if successors is empty, return failure & inf
    if not successors:  
        return None, float('inf')
    #update f with value from previous search
    for s in successors:     
        s[4] = max(s[4], f)
    #loop until return    
    while True:
        #sort the list by updated f of each successor
        newlist = sorted(successors, key = lambda x: x[4]) 
        best = newlist[0]
        if best[4] > f_limit: #best[4] is best_f
            return None, best[4]
        alternative = newlist[1] #get 2nd-lowest f-value node
        result, best[4] = rbfs(problem, best, min(f_limit, alternative[4]), heuristic)
        #return result and None(for f-cost)
        if result != None:  return result, None
    
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
rebfs = recursivebfs

# searchAgents.py
# ---------------
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
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

from game import Directions
from game import Agent
from game import Actions
import util
import time
import search

class GoWestAgent(Agent):
    "An agent that goes West until it can't."

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP

#######################################################
# This portion is written for you, but will only work #
#       after you fill in parts of search.py          #
#######################################################

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search
    algorithm for a supplied search problem, then returns actions to follow that
    path.

    As a default, this agent runs DFS on a PositionSearchProblem to find
    location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems
        
        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError(fn + ' is not a search function in search.py.')
        func = getattr(search, fn)
        if 'heuristic' not in func.__code__.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError(heuristic + ' is not a function in searchAgents.py or search.py.')
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError(prob + ' is not a search problem type in SearchAgents.py.')
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception("No search function provided for SearchAgent")
        starttime = time.time()
        problem = self.searchType(state) # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        totalCost = problem.getCostOfActionSequence(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP

class DeceptiveSearchAgentpi2(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        #COMP90054 Task 4 - Implement your deceptive search algorithm here
        prob = FoodSearchProblem(state)
        s = prob.getStartState()   #start node s in piD2 problem
        position, foodGrid, capsules = s
        foods = foodGrid.asList()     
        
        gr = foods[0]    #assume there's only one goal                       
        #optimal cost are manhattan distance
        optc_s_gr = util.manhattanDistance(position, gr) 
        optc_s_gi = []                         
        optc_gr_gi = []                         
        #calcualte manhattan distance from position to gi, gr to gi
        for cap in capsules:
            optc_s_gi.append(util.manhattanDistance(position,cap))
            optc_gr_gi.append(util.manhattanDistance(gr,cap))
            
        # get beta_min and corresponding g_min
        beta = [] 
        for i in range(len(capsules)):
            beta.append((optc_gr_gi[i] + optc_s_gr - optc_s_gi[i]) / 2)       
        beta_min = min(beta)
        optc_gr_gmin = optc_gr_gi[beta.index(beta_min)]
        g_min = capsules[beta.index(beta_min)]

        #--------------------Finding t node---------------------#
        #start from s node to g_min
        open_list = util.PriorityQueue()
        startNode = (s, '', 0, [(s,'')])
        open_list.push(startNode, 0) 
        closed_state_list = set()
        while not open_list.isEmpty():
            currentNode = open_list.pop()
            currState, currAct, currCost, currPath = currentNode
            currPosition, currFoodGrid, currCaps = currState
            #when reaches g_min
            if currPosition == g_min:
                #start g_min to gr direction
                open_list = util.PriorityQueue()
                #reset start node as g_min
                startNode = (currState, '', 0, [(currState,'')])
                open_list.push(startNode, 0) 
                closed_state_list = set()    
                while not open_list.isEmpty():
                    currentNode = open_list.pop()
                    currState, currAct, currCost, currPath = currentNode
                    currPosition, currFoodGrid, currCaps = currState  
                    #when optc(currentNode, gr) is beta_min
                    if optc_gr_gmin - currCost == beta_min and (currPosition[0]==gr[0] or currPosition[1]==gr[1]):
                        t = currState
                        break
                    if currState not in closed_state_list:
                        closed_state_list.add(currState)          
                        succNodes = prob.expand(currState)
                        for succNode in succNodes:
                            succState, succAction, succCost = succNode
                            if succState not in closed_state_list:
                                newNode = ( succState, succAction, currCost+succCost, currPath+[(succState, succAction)])
                                fCost = currCost + succCost + util.manhattanDistance(succState[0] , gr)
                                open_list.push(newNode, fCost) 
                break
            if currState not in closed_state_list:
                closed_state_list.add(currState) 
                succNodes = prob.expand(currState)
                for succNode in succNodes:
                    succState, succAction, succCost = succNode
                    if succState not in closed_state_list:
                        newNode = ( succState, succAction, currCost+succCost, currPath+[(succState, succAction)])
                        fCost = currCost + succCost + util.manhattanDistance(succState[0] , gr)
                        open_list.push(newNode, fCost)
        #------------------------------------------------------#

        #get path from A* algorithm
        s_to_t = getPath(s, t, prob)
        #find path from t to gr
        t_to_gr = getPath(t, None, prob)
        #append two paths together
        full_path = s_to_t + t_to_gr
        print(full_path)
        #assign to self.actions
        self.actions = full_path
        return self.actions

class DeceptiveSearchAgentpi3(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        #COMP90054 Task 4 - Implement your deceptive search algorithm here
        prob = FoodSearchProblem(state)
        s = prob.getStartState()   #start node s in piD2 problem
        position, foodGrid, capsules = s
        foods = foodGrid.asList()     
        
        gr = foods[0]   #assume there's only one food                        
        #optimal cost are manhattan distance
        optc_s_gr = util.manhattanDistance(position, gr) 
        optc_s_gi = []                         
        optc_gr_gi = []                         
        #calcualte manhattan distance from position to gi, gr to gi
        for cap in capsules:
            optc_s_gi.append(util.manhattanDistance(position,cap))
            optc_gr_gi.append(util.manhattanDistance(gr,cap))
            
        # get beta_min and corresponding g_min
        beta = [] 
        for i in range(len(capsules)):
            beta.append((optc_gr_gi[i] + optc_s_gr - optc_s_gi[i]) / 2)       
        beta_min = min(beta)
        optc_gr_gmin = optc_gr_gi[beta.index(beta_min)]
        g_min = capsules[beta.index(beta_min)]

        #--------------------Finding t node---------------------#
        #start from s node to g_min
        open_list = util.PriorityQueue()
        startNode = (s, '', 0, [(s,'')])
        open_list.push(startNode, 0) 
        closed_state_list = set()
        while not open_list.isEmpty():
            currentNode = open_list.pop()
            currState, currAct, currCost, currPath = currentNode
            currPosition, currFoodGrid, currCaps = currState
            #when reaches g_min
            if currPosition == g_min:
                actions = [x[1] for x in currPath]
                del actions[0]
                s_gmin = actions
                #start g_min to gr direction
                open_list = util.PriorityQueue()
                #reset start node as g_min
                startNode = (currState, '', 0, [(currState,'')])
                open_list.push(startNode, 0) 
                closed_state_list = set()    
                while not open_list.isEmpty():
                    currentNode = open_list.pop()
                    currState, currAct, currCost, currPath = currentNode
                    currPosition, currFoodGrid, currCaps = currState  
                    # when optc(currentNode, gr) is beta_min
                    if optc_gr_gmin - currCost == beta_min and (currPosition[0]==gr[0] or currPosition[1]==gr[1]):
                        t = currState #get t node
                        actions = [x[1] for x in currPath]
                        del actions[0]
                        gmin_gr = actions
                        break
                    if currState not in closed_state_list:
                        closed_state_list.add(currState)          
                        succNodes = prob.expand(currState)
                        for succNode in succNodes:
                            succState, succAction, succCost = succNode
                            if succState not in closed_state_list:
                                newNode = ( succState, succAction, currCost+succCost, currPath+[(succState, succAction)])
                                fCost = currCost + succCost + util.manhattanDistance(succState[0] , gr)
                                open_list.push(newNode, fCost) 
                break
            if currState not in closed_state_list:
                closed_state_list.add(currState) 
                succNodes = prob.expand(currState)
                for succNode in succNodes:
                    succState, succAction, succCost = succNode
                    if succState not in closed_state_list:
                        newNode = ( succState, succAction, currCost+succCost, currPath+[(succState, succAction)])
                        fCost = currCost + succCost + util.manhattanDistance(succState[0] , gr)
                        open_list.push(newNode, fCost)
        #--------------------------------------------------------#
        
        # reaching gr by adjusted heurstic value
        open_list = util.PriorityQueue()
        startNode = (s, '', 0, [(s,'')])
        open_list.push(startNode, 0) 
        closed_state_list = set() 
        while not open_list.isEmpty():
            currentNode = open_list.pop()
            currState, currAct, currCost, currPath = currentNode
            currPosition, currFoodGrid, currCaps = currState
            #if reaches gr
            if currPosition == gr:
                actions = [x[1] for x in currPath]
                del actions[0] 
                finalpath = actions
                print("actions: ",finalpath)
                break
            if currState not in closed_state_list:
                closed_state_list.add(currState) 
                succNodes = prob.expand(currState)
                for succNode in succNodes:
                    succState, succAction, succCost = succNode
                    if succState not in closed_state_list:
                        newNode = ( succState, succAction, currCost+succCost, currPath+[(succState, succAction)])
                        h_value = util.manhattanDistance(succState[0], gr)
                        #if h(n, gr) < h(n, gmin)
                        if util.manhattanDistance(succState[0],gr) < util.manhattanDistance(succState[0],g_min):
                            h_value *= 3 # multiply a random constant to its original heuristic
                        fCost = currCost + succCost +h_value
                        open_list.push(newNode, fCost)  

        self.actions = finalpath
        return self.actions 

#self defined funtion returning path from start to end node
def getPath(start, end, problem): #requires start and end state node
    open_list = util.PriorityQueue()
    startNode = (start, '', 0, [(start,'')])
    open_list.push(startNode, 0)
    closed_state_list = set()
    while not open_list.isEmpty():
        currentNode = open_list.pop() 
        state, act, cost, path = currentNode
        if (end != None and state[0] == end[0]) or (end == None and problem.isGoalState(state)):
                actions = [x[1] for x in path]
                del actions[0] 
                return actions     
        if state not in closed_state_list:
            closed_state_list.add(state) 
            succNodes = problem.expand(state)
            for succNode in succNodes:
                succState, succAction, succCost = succNode
                if succState not in closed_state_list:
                    newNode = ( succState, succAction, cost+succCost, path+[(succState, succAction)])
                    fCost = cost + succCost
                    open_list.push(newNode, fCost) 
    return None
class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, child
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def expand(self, state):
        """
        Returns child states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (child, action, stepCost), where 'child' is a
         child to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that child
        """

        children = []
        for action in self.getActions(state):
            nextState = self.getNextState(state, action)
            cost = self.getActionCost(state, action, nextState)
            children.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return children

    def getActions(self, state):
        possible_directions = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
        valid_actions_from_state = []
        for action in possible_directions:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                valid_actions_from_state.append(action)
        return valid_actions_from_state

    def getActionCost(self, state, action, next_state):
        assert next_state == self.getNextState(state, action), (
            "Invalid next state passed to getActionCost().")
        return self.costFn(next_state)

    def getNextState(self, state, action):
        assert action in self.getActions(state), (
            "Invalid action passed to getActionCost().")
        x, y = state
        dx, dy = Actions.directionToVector(action)
        nextx, nexty = int(x + dx), int(y + dy)
        return (nextx, nexty)

    def getCostOfActionSequence(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost

class StayEastSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the West side of the board.

    The cost function for stepping into a position (x,y) is 1/2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)

class StayWestSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the East side of the board.

    The cost function for stepping into a position (x,y) is 2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersProblem(search.SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and child function
    """

    def __init__(self, startingGameState):
        """
        Stores the walls, pacman's starting position and corners.
        """
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height-2, self.walls.width-2
        self.corners = ((1,1), (1,top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print('Warning: no food in corner ' + str(corner))
        self._expanded = 0 # DO NOT CHANGE; Number of search nodes expanded
        # Please add any code here which you would like to use
        # in initializing the problem
        "*** YOUR CODE HERE ***"
        

    def getStartState(self):
        """
        Returns the start state (in your state space, not the full Pacman state
        space)
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
        Returns whether this search state is a goal state of the problem.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

    def expand(self, state):
        """
        Returns child states, the actions they require, and a cost of 1.

         As noted in search.py:
            For a given state, this should return a list of triples, (child,
            action, stepCost), where 'child' is a child to the current
            state, 'action' is the action required to get there, and 'stepCost'
            is the incremental cost of expanding to that child
        """

        children = []
        for action in self.getActions(state):
            # Add a child state to the child list if the action is legal
            # You should call getActions, getActionCost, and getNextState.
            "*** YOUR CODE HERE ***"

        self._expanded += 1 # DO NOT CHANGE
        return children

    def getActions(self, state):
        possible_directions = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
        valid_actions_from_state = []
        for action in possible_directions:
            x, y = state[0]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                valid_actions_from_state.append(action)
        return valid_actions_from_state

    def getActionCost(self, state, action, next_state):
        assert next_state == self.getNextState(state, action), (
            "Invalid next state passed to getActionCost().")
        return 1

    def getNextState(self, state, action):
        assert action in self.getActions(state), (
            "Invalid action passed to getActionCost().")
        x, y = state[0]
        dx, dy = Actions.directionToVector(action)
        nextx, nexty = int(x + dx), int(y + dy)
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()
        # you will need to replace the None part of the following tuple.
        return ((nextx, nexty), None)

    def getCostOfActionSequence(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions == None: return 999999
        x,y= self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)


def cornersHeuristic(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    "*** YOUR CODE HERE ***"
    return 0 # Default to trivial solution

class AStarCornersAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem

class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid, capsules ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
      capsules:       a tuple containing tuples (x,y) that specify the location of each capsule
    """
    def __init__(self, startingGameState):
        # Note our starting state now includes the Capsule positions
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood(), tuple(startingGameState.getCapsules()))
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0 # DO NOT CHANGE
        self.heuristicInfo = {} # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def expand(self, state):
        "Returns child states, the actions they require, and a cost of 1."
        children = []
        self._expanded += 1 # DO NOT CHANGE
        for action in self.getActions(state):
            next_state = self.getNextState(state, action)
            action_cost = self.getActionCost(state, action, next_state)
            children.append( ( next_state, action, action_cost) )
        return children

    def getActions(self, state):
        possible_directions = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
        valid_actions_from_state = []
        for action in possible_directions:
            x, y = state[0]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                valid_actions_from_state.append(action)
        return valid_actions_from_state

    def getActionCost(self, state, action, next_state):
        assert next_state == self.getNextState(state, action), (
            "Invalid next state passed to getActionCost().")
        dx, dy = Actions.directionToVector(action)
        x,y = state[0]
        x, y = int(x + dx), int(y + dy)
        position = x,y
        if( position in list(state[2]) ):
            return 0
        else :
            return 1

    def getNextState(self, state, action):
        assert action in self.getActions(state), (
            "Invalid action passed to getActionCost().")
        x, y = state[0]
        dx, dy = Actions.directionToVector(action)
        nextx, nexty = int(x + dx), int(y + dy)
        nextFood = state[1].copy()
        nextFood[nextx][nexty] = False
        nextCapsules = list(state[2])
        position = nextx, nexty
        if position in nextCapsules :
            nextCapsules.remove(position)
        nextCapsules = tuple(nextCapsules)
        # Our new state information now contains Capsule locations
        return ((nextx, nexty), nextFood, nextCapsules)

    def getCostOfActionSequence(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        state = self.startingGameState
        x,y= self.getStartState()[0]
        capsules = list(self.getStartState()[2])
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999         
            position = x,y
            if( position in capsules):
                capsules.remove(position)
            else :
                cost += 1
        return cost

class AStarFoodSearchAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem

def get_distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return abs(x1-x2) + abs(y1-y2)
  
def get_min(queue) :
	cost = 9999999
	cheapest = -1
	for index, item in enumerate(queue) :
		if item[1] < cost :
			cost = item[1]
			cheapest = index	
	cheapestItem = queue[cheapest]
	del queue[cheapest]
	return cheapestItem

counter = 0
def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be admissible to ensure correctness.

    The state is a tuple ( pacmanPosition, foodGrid, capsules) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.  capsules contains a tuple of capsule locations.

    If you want access to info like walls, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid, capsules = state
    #COMP90054 Task 3, Implement your code here
    foods = foodGrid.asList() #get a list of food corrdinates

    global counter 
    counter += 1
    print("heurstic ",counter)

    #return 0 if reaches goal state
    if problem.isGoalState(state):
        return 0

    #manhattan distance (position -> foods)
    food_dist = []                  
    for each_food in foods:
        food_dist.append(util.manhattanDistance(position,each_food))
    
    h = max(food_dist)              #default h-value
    #goal counting
    h2 = len(foods)

    # when capsules[] is not empty
    if len(capsules) != 0:
        if len(foods) < 1:
            # Using BFS to reach goal
            myqueue = util.Queue()
            startNode = (problem.getStartState(), '', 0, [])
            myqueue.push(startNode)
            visited = set()
            while myqueue :
                node = myqueue.pop()
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
                        myqueue.push(newNode)
            #get h value for bfs
            bfs_h = len(path)
            if bfs_h < h2:                     
                print("bfs_h: ", bfs_h)


        furthest_food_cap = []      #manhattan distance (furthest food -> capsules)
        cap_dist = []               #manhattan distance (position -> capsules)
        #get furthest food through index
        furthest_food = foods[food_dist.index(max(food_dist))]
        for each_cap in capsules:
            cap_dist.append(util.manhattanDistance(each_cap,position))
            #append distance of each capsule to the furthest food
            furthest_food_cap.append(util.manhattanDistance(furthest_food,each_cap))
        # h1 = closet cap + closet food + furthest food to the mean-value distance of capsules
        h1 = min(cap_dist) + min(food_dist) + furthest_food_cap[int(len(furthest_food_cap)*0.58)]
        

        print("\n")
        return min(h1,h2)
        
    else: return h
    

        
    
   

class ClosestDotSearchAgent(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while(currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState) # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception('findPathToClosestDot returned an illegal move: %s!\n%s' % t)
                currentState = currentState.generateChild(0, action)
        self.actionIndex = 0
        print('Path found with cost %d.' % len(self.actions))

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)

        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    child function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))

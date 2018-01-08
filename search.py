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

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    """
    print "Start:", problem.getStartState()
    print "Is the start a goal?"
    problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())"""

    stacktobevisited = util.Stack()
    stack2 = util.Stack()
    control = problem.getStartState()
    while not problem.isGoalState(control):
        for successors in problem.getSuccessors(control):
            notVisited = True
            for s in stack2.list:
                if successors[0] == s[0]:
                    notVisited = False
            if not successors[0] == problem.getStartState() and notVisited:
                stacktobevisited.push((successors[0],successors[1],control))
        control = stacktobevisited.pop()
        stack2.push(control)
        control = control[0]


    lastNode = stack2.pop()
    path = []
    path = [lastNode[1]] + path
    while not stack2.isEmpty():
        node = stack2.pop()
        if lastNode[2] == node[0]:
            path = [node[1]] + path
            lastNode = node
    return path

    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    stacktobevisited = util.Queue()
    stack2 = util.Stack()
    control = problem.getStartState()
    while not problem.isGoalState(control):
        for successors in problem.getSuccessors(control):
            notVisited = True
            for s in stack2.list:
                if successors[0] == s[0]:
                    notVisited = False
            nottobevisited = True
            for s in stacktobevisited.list:
                if successors[0] == s[0]:
                    nottobevisited = False
            if not successors[0] == problem.getStartState() and notVisited and nottobevisited:
                stacktobevisited.push((successors[0],successors[1],control))
        control = stacktobevisited.pop()
        stack2.push(control)
        control = control[0]
    lastNode = stack2.pop()
    path = []
    path = [lastNode[1]] + path
    while not stack2.isEmpty():
        node = stack2.pop()
        if lastNode[2] == node[0]:
            path = [node[1]] + path
            lastNode = node
    return path
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    stacktobevisited = util.PriorityQueue()
    stack2 = util.Stack()
    control = problem.getStartState()
    dictionary = {}
    dictionary[control] = 0
    pathDict = {}
    pathDict[control] = (0,0)
    while not problem.isGoalState(control):
        for successors in problem.getSuccessors(control):
            notVisited = True
            if successors[0] in stack2.list:
                    notVisited = False
            if not successors[0] == problem.getStartState() and notVisited:
                cost = successors[2] + dictionary[control]
                stacktobevisited.update(successors[0], cost)
                if (dictionary.__contains__(successors[0]) and dictionary[successors[0]] > cost) or not dictionary.__contains__(successors[0]):
                    dictionary[successors[0]] = cost
                    pathDict[successors[0]] = (control, successors[1])
        control = stacktobevisited.pop()
        stack2.push(control)

    lastNode = stack2.pop()
    path = []
    while not lastNode == problem.getStartState():
        path = [pathDict[lastNode][1]] + path
        lastNode = pathDict[lastNode][0]
    return path


    "*** YOUR CODE HERE ***"

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    stacktobevisited = util.PriorityQueue()
    stack2 = util.Stack()
    control = problem.getStartState()
    dictionary = {}
    dictionary[control] = heuristic(control, problem)
    pathDict = {}
    pathDict[control] = (0, 0)
    while not problem.isGoalState(control):
        for successors in problem.getSuccessors(control):
            notVisited = True
            if successors[0] in stack2.list:
                notVisited = False
            if not successors[0] == problem.getStartState() and notVisited:
                cost = successors[2] + dictionary[control] - heuristic(control, problem) + heuristic(successors[0], problem)
                stacktobevisited.update(successors[0], cost)
                if (dictionary.__contains__(successors[0]) and dictionary[successors[0]] > cost) or not dictionary.__contains__(successors[0]):
                    dictionary[successors[0]] = cost
                    pathDict[successors[0]] = (control, successors[1])
        control = stacktobevisited.pop()
        stack2.push(control)

    lastNode = stack2.pop()
    path = []
    while not lastNode == problem.getStartState():
        path = [pathDict[lastNode][1]] + path
        lastNode = pathDict[lastNode][0]
    return path
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

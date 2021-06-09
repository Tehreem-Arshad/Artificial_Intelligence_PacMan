# 21100250 Tehreem Arshad PACMAN IMPLEMENTATION
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

from queue import PriorityQueue

def GraphSearch (problem, fringe):
	node = problem.getStartState()
	fringe.push([(node,"Stop",0)]) # ([(state,action,cost)])
	explored = [] #Initializing empty list for explored nodes
	while not fringe.isEmpty():
		route = fringe.pop() #Get the path or route
		CurrentState = route[-1][0] #Current state will be the first element of the last tuple
		if problem.isGoalState(CurrentState):
			return [x[1] for x in route][1:] #Return second element(action) for every tuple in the route
		if CurrentState not in explored:
			explored.append(CurrentState) #If state not visited than mark it visited
			for successor in problem.getSuccessors(CurrentState):
				if successor[0]  not in explored: #[0]-> successor state
					succ_route = route[:] #copy parent's route
					succ_route.append(successor) # Successor route = parent's route + successors'
					fringe.push(succ_route)

	return fail #If search fails


def depthFirstSearch(problem):
	"""
	Search the deepest nodes in the search tree first.

	Your search algorithm needs to return a list of actions that reaches the
	goal. Make sure to implement a graph search algorithm.

	To get started, you might want to try some of these simple commands to
	understand the search problem that is being passed in:
		"""
	#print("Start:", problem.getStartState())
	#print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
	#print("Start's successors:", problem.getSuccessors(problem.getStartState()))
	#print (problem)
	"Start of your code here"
	frontier = util.Stack()
	return GraphSearch(problem, frontier)
	pass
	"End of your code"                

	


	

		
	
                
                        
# ________________________________________________________________

class _RecursiveDepthFirstSearch(object):
	'''
		=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
		above. 
		Key Point: Remember in tutorial you were asked to expand the left-most child 
		first for dfs and bfs for consistency. If you expanded the right-most
		first, dfs/bfs would be correct in principle but may not return the same
		path. 

		=> Useful Hint: self.problem.getSuccessors(node) will return children of 
		a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
		is different from 'iterative' traversal, try reversing the sequence.  

	'''
	def __init__(self, problem):
		" Do not change this. " 
		# You'll save the actions that recursive dfs found in self.actions. 
		self.actions = [] 
		# Use self.explored to keep track of explored nodes.  
		self.explored = set()
		self.problem = problem

	def RecursiveDepthFirstSearchHelper(self, node):
		'''
		args: start node 
		outputs: bool => True if path found else Fasle.
		'''
		"Start of Your Code"
		if self.problem.isGoalState(node):
			return True
		children = self.problem.getSuccessors(node)
		self.explored.add(node)
		rev = reversed (children)
		new_list = list (children)
		for child in new_list:
			child_state = child[0]
			child_path = child[1]
			visited = False
			for any_state in self.explored:
				if any_state == child_state:
					visited = True
					break
				else:
					continue
			if visited == False:
				recursive_call = self.RecursiveDepthFirstSearchHelper(child_state)
				if recursive_call == True:
					self.actions = self.actions + [child_path]
					return True
				else:
					self.actions = []
		return False




		pass
		"End of Your Code"


def RecursiveDepthFirstSearch(problem):
	" You need not change this function. "
	# All your code should be in member function 'RecursiveDepthFirstSearchHelper' of 
	# class '_RecursiveDepthFirstSearch'."

	node = problem.getStartState() 
	rdfs = _RecursiveDepthFirstSearch(problem)
	path_found = rdfs.RecursiveDepthFirstSearchHelper(node)

	return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def depthLimitedSearch(problem, limit = 129): #For Big maze 222, For medium maze 129

	"""
	Search the deepest nodes in the search tree first as long as the
	nodes are not not deeper than 'limit'.

	For medium maze, pacman should find food for limit less than 130. 
	If your solution needs 'limit' more than 130, it's bogus.
	Specifically, for:
	'python pacman.py -l mediumMaze -p SearchAgent -a fn=dls', and limit=130
	pacman should work normally.  

	Your search algorithm needs to return a list of actions that reaches the
	goal. Make sure to implement a graph search algorithm.
	Autograder cannot test this function.  

	Hints: You may need to store additional information in your frontier(queue).

		"""
	
	"Start of Your Code"

	frontier = util.Stack()
	explored = set()
	actions = [] 
	depth = 0

	start_state = problem.getStartState()

	if problem.isGoalState(start_state):
		return []

	start_node = (start_state, [], depth)

	frontier.push(start_node)

	while not frontier.isEmpty():
	    
	   if frontier.isEmpty():
	   		return []

	   if depth > limit:
	        return []

	   node = frontier.pop()
	   state = node[0]
	   actions = node[1]
	   depth = node[2]
	   if state not in explored:
	        explored.add(state)
	   if problem.isGoalState(state):
	        return actions

	   for child in problem.getSuccessors(state):
	   	child_state = child[0]
	   	child_path = child [1]
	   	if child_state not in explored:
	   		curr_depth = depth + 1
	   		data = (child_state, actions + [child_path], curr_depth) 
	   		frontier.push(data)
	   	else:
	   		continue
	pass

	"End of Your Code"
# ________________________________________________________________

class _RecursiveDepthLimitedSearch(object):
	'''
		=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
		above. 
		Key Point: Remember in tutorial you were asked to expand the left-most child 
		first for dfs and bfs for consistency. If you expanded the right-most
		first, dfs/bfs would be correct in principle but may not return the same
		path. 

		=> Useful Hint: self.problem.getSuccessors(node) will return children of 
		a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
		is different from 'iterative' traversal, try reversing the sequence.  

	'''
	def __init__(self, problem):
		" Do not change this. " 
		# You'll save the actions that recursive dfs found in self.actions. 
		self.actions = [] 
		# Use self.explored to keep track of explored nodes.  
		self.explored = set()
		self.problem = problem
		self.current_depth = 0
		self.depth_limit = 204 # For medium maze, You should find solution for depth_limit not more than 204.

	def RecursiveDepthLimitedSearchHelper(self, node):
		'''
		args: start node 
		outputs: bool => True if path found else Fasle.
		'''

		"Start of Your Code"
		self.explored.add(node) 

		if self.problem.isGoalState(node):
		    return True
		DepthNow = self.current_depth
		DepthLimit = self.depth_limit
		visited = self.explored
		if DepthNow > DepthLimit:
		    return[]
		Descedents = self.problem.getSuccessors(node)
		reverse_it = reversed (Descedents)
		for state, path, _ in reverse_it: 
		    if state not in visited:
		        DepthNow = DepthNow + 1
		        if self.RecursiveDepthLimitedSearchHelper(state):
		            self.actions.append(path)
		            return True
		DepthNow = DepthNow - 1
		return False 	
		pass
		"End of Your Code"


def RecursiveDepthLimitedSearch(problem):
	"You need not change this function. All your code in member function RecursiveDepthLimitedSearchHelper"
	node = problem.getStartState() 
	rdfs = _RecursiveDepthLimitedSearch(problem)
	path_found = rdfs.RecursiveDepthLimitedSearchHelper(node)
	return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def breadthFirstSearch(problem):
	"""Search the shallowest nodes in the search tree first."""

	"Start of Your Code"
	frontier = util.Queue()
	return GraphSearch (problem, frontier)
	pass
	"End of Your Code"


def uniformCostSearch(problem):
	"""Search the node of least total cost first.
	   You may need to pay close attention to util.py.
	   Useful Reminder: Note that problem.getSuccessors(node) returns "step_cost". 

	   Key Point: If a node is already present in the queue with higher path cost 
	   (or higher priority), you'll update its cost (or priority) 
	   (Similar to pseudocode in figure 3.14 of your textbook.). 
	   Be careful, autograder cannot catch this bug.
	"""

	"Start of Your Code"
	frontier = util.PriorityQueue()
	frontier.push((problem.getStartState(),[]),0)
	explored = []
	while True:
		if frontier.isEmpty():
			break
		node,actions = frontier.pop()
		curr_state = node[0] 
		curr_path = node[1]
		goal = problem.isGoalState(curr_state) 
		if goal == True:
			return curr_path
		if curr_state not in explored:
			successors = problem.getSuccessors(curr_state)
			for succ_nodes in successors:
				succ_state = succ_nodes[0]
				succ_path  = succ_nodes[1]
				Already_visited = False
				for any_state in explored:
					if any_state == succ_state:
						Already_visited = True
						break
					else:
						continue
				if Already_visited == False:
					final_path = curr_path + [succ_path]
					cost = problem.getCostOfActions(final_path)
					frontier.push((succ_state, final_path), cost)
			explored.append(curr_state)
	return final_path
	pass
	"End of Your Code"

def nullHeuristic(state, problem=None):
	"""
	A heuristic function estimates the cost from the current state to the nearest
	goal in the provided SearchProblem.  This heuristic is trivial.
	"""
	return 0

def aStarSearch(problem, heuristic=nullHeuristic):
	'''
	Pay clos attention to util.py- specifically, args you pass to member functions. 

	Key Point: If a node is already present in the queue with higher path cost 
	(or higher priority), you'll update its cost (or priority) 
	(Similar to pseudocode in figure 3.14 of your textbook.). 
	Be careful, autograder cannot catch this bug.

	'''
	"Start of Your Code"
	frontier = util.PriorityQueue()
	frontier.push((problem.getStartState(),[]),0)
	explored = []
	while True:
		if frontier.isEmpty():
			break
		node,actions = frontier.pop()
		curr_state = node[0] 
		curr_path = node[1]
		goal = problem.isGoalState(curr_state) 
		if goal == True:
			return curr_path
		if curr_state not in explored:
			successors = problem.getSuccessors(curr_state)
			for succ_nodes in successors:
				succ_state = succ_nodes[0]
				succ_path  = succ_nodes[1]
				Already_visited = False
				for any_state in explored:
					if any_state == succ_state:
						Already_visited = True
						break
					else:
						continue
				if Already_visited == False:
					final_path = curr_path + [succ_path]
					g = problem.getCostOfActions(final_path)
					h= heuristic(succ_state,problem)
					f = g+h
					frontier.push((succ_state, final_path), f)
			explored.append(curr_state)
	return final_path
	pass
	
	pass
	"End of Your Code"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
rdfs = RecursiveDepthFirstSearch
dls = depthLimitedSearch
rdls = RecursiveDepthLimitedSearch
astar = aStarSearch
ucs = uniformCostSearch

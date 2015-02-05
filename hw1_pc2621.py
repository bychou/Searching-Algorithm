

__author__ = "Ilambharathi Kanniah"
__email__ = "ik2342@columbia.edu"

import argparse
import sys
import copy
import time
import heapq

parser = argparse.ArgumentParser(description='Robot Path Planning | HW 1 | COMS 4701')
parser.add_argument('-bfs', action="store_true", default=False , help="Run BFS on the map")
parser.add_argument('-dfs', action="store_true", default=False, help= "Run DFS on the map")
parser.add_argument('-astar', action="store_true", default=False, help="Run A* on the map")
parser.add_argument('-ida', action="store_true", default=False, help="Run IDA on the map")
parser.add_argument('-all', action="store_true", default=False, help="Run all the 3 algorithms")
parser.add_argument('-m', action="store", help="Map filename")

results = parser.parse_args()

if results.m=="" or not(results.all or results.astar or results.bfs or results.dfs or results.ida):
	print "Check the parameters : >> python hw1_UNI.py -h"
	exit()

OUTPUT_FILENAME = "output%s.txt" %results.m[5]
if results.all:
	results.bfs = results.dfs = results.astar = results.ida = True

# Reading of map given and all other initializations
try:
    with open(results.m) as f:
        arena = f.read()
        arena = arena.split("\n")[:-1]
except:
	print "Error in reading the arena file."
	exit()
  
print "mapSize:" , len(arena)*len(arena[0])

# Convert the arena from list of string into a 2D list, and search for source 
# and goal position. 
for i in range(0,len(arena)):
    
    chars = []
    for j in range(0,len(arena[i])):

        chars.append(arena[i][j])

        if (arena[i][j] == 'g'):
        	goal = [j,i]
       	elif (arena[i][j] == 's'):
       		source = [j,i]

    arena[i] = chars

# These four functions check is you can move to adjacent nodes
def canUp (node):
	return not ((node[1] - 1 < 0) or (arena[node[1] - 1][node[0]] == 'o'))
def canDown (node):
	return not ((node[1] + 1 > len(arena) - 1) or (arena[node[1] + 1][node[0]] == 'o'))
def canLeft (node):
	return not ((node[0] - 1 < 0) or (arena[node[1]][node[0] - 1] == 'o'))
def canRight (node):
	return not ((node[0] + 1 > len(arena[0]) - 1) or (arena[node[1]][node[0] + 1] == 'o'))

# Heuristic function for A8 search, it calculate the distance between two points
def heuristic (nodeA, nodeB):
	return abs(nodeA[0] - nodeB[0]) + abs(nodeA[1] - nodeB[1])

# Given a solution arena with path printed with "*"
# it will first convert the map from 2D list to list of string
# and print the solution, path, the write it to file
def writeFile (solArena, message):
	for i in range(len(solArena)):
		s = ''.join(solArena[i])
		solArena[i] = s
	f = open(OUTPUT_FILENAME,"a")
	for i in solArena:
		f.write("%s\n" %i)
		print i
	f.write(message)
	print message
	f.close()



if results.bfs:

	print "====================BFS===================="
	start_time = time.time()


	# Put source node into the frontier
	# Frontier is a 1D list, storing the frontier nodes[[x1,y1],[x2,y2],...]
	# explored is a 2D list, storing the parent position of the explored node
	# ex: if explored[i][j] = [x,y] => the parent of node [i,j] is [x,y] 
	frontier = [source]
	explored = [[[] for i in range(len(arena))] for j in range(len(arena[0]))] 


	finish = False
	# start searching
	while not finish:

		if not frontier:
			print "Failed to find a solution via bfs..."
			break 

		# Pop a node from the frontier, and expand its child nodes
		node = frontier.pop(0)

		# Find available child nodes
		childs = []
		[x,y] = node

		if canUp(node):
			childs.append([x, y - 1])
		if canDown(node):
			childs.append([x, y + 1])
		if canLeft(node):
			childs.append([x - 1, y])
		if canRight(node):
			childs.append([x + 1, y])

		for child in childs:

			[x,y] = [child[0] , child[1]]

			# If the childs is not explored, put it into the frontier and mark its parent in the explored array
			if (not explored[x][y]):

				frontier.append(child)
				explored[x][y] = node

			# Success condition
			if (child == goal):
				finish = True
				print "Succeed in finding a path via bfs, writing solution to \"%s\"..." %OUTPUT_FILENAME
				
		if (finish):
			break


	
	if (finish):

		# Finding the path and create a new arena "solArena" to plot the result
		solArena = copy.deepcopy(arena)
		[x,y] = [node[0] , node[1]]
		path_cost = 1;
		while source != [x,y] :

			solArena[y][x] = '*'
			[prevX, prevY] = [x,y]
			[x,y] = explored[prevX][prevY]
			path_cost += 1

		writeFile(solArena, "BFS: %d\n" %path_cost)
		print "Time taken:" , time.time() - start_time , "second(s)..."


if results.dfs:

	print "====================DFS===================="
	start_time = time.time()

	sys.setrecursionlimit(10000)

	# Using a recursive function visit to perform dfs

	def visit(node):

		link.append(node)
		[x,y] = node


		if node == goal:
			
			return True


		# Check available child nodes, it should not be in "explored" and "link"
		# If visit function return true, then return true for this parent function
		if canUp(node) and [x, y - 1] not in explored and [x, y - 1] not in link and visit([x, y - 1]):
		 	return True
		if canDown(node) and [x, y + 1] not in explored and [x, y + 1] not in link and visit([x, y + 1]):
		 	return True
		if canLeft(node) and [x - 1, y] not in explored and [x - 1, y] not in link and visit([x - 1, y]):
			return True
		if canRight(node) and [x + 1, y] not in explored and [x + 1, y] not in link and visit([x + 1, y]):
			return True
		
		explored.append(node)
		# After finish visit all its child nodes but can not succeed, 
		# pop the node out the "link" and return false => this path is not available
		link.pop()
		return False


	# Push soruce node into the "link", and initial "explored"
	# "explored" is used to track duplicate search
	link = [source]
	explored = []

	[x,y] = source

	# If one of the child nodes succeed, return and writing the solution
	if canUp(source) and visit([x, y - 1]) or canDown(source) and visit([x, y + 1]) or canLeft(source) and visit([x - 1, y]) or canRight(source) and visit([x + 1, y]):
		
		# If suceed, convert arena to list of string and write file
		print "Succeed in finding a solution via dfs, writing solution to \"%s\"..." %OUTPUT_FILENAME
		solArena = copy.deepcopy(arena)
		for i in range(1,len(link) - 1):
			solArena[link[i][1]][link[i][0]] = '*' 
		for i in range(len(solArena)):
			s = ''.join(solArena[i])
			solArena[i] = s

		writeFile(solArena, "DFS: %d\n"  %(len(link) - 2))
		print "Time taken:" , time.time() - start_time , "second(s)..."

	else:
		# If all recursive call failed, print "Failed"
		print "Failed to find a solution via dfs"



if results.astar:

	print "====================A*===================="
	start_time = time.time()


	# frontier is a heap queue to store the cost of node
	# cost = g + h, for source g = 0
	# Put the heuristic function value of the source node into the frontier.
	frontier = []
	heapq.heappush(frontier, [heuristic(source, goal), source])
	explored = [[[] for i in range(len(arena))] for j in range(len(arena[0]))] 

	
	# explored is a 2D array to keep track of visited nodes
	explored[source[0]][source[1]] = ["source"] 

	finish = False
	
	while not finish:
		
		if not frontier:
			print "Failed to find a solution via A* ..."

		[cost, node] = heapq.heappop(frontier)

		h = heuristic(node, goal)
		g = cost - h 

		# Calculate the g value of child (one more step than its parent)
		g_child = g + 1 

		
		childs = []
		[x,y] = [node[0] , node[1]]

		# Check available child nodes and calculate the cost
		# childs = [[cost, [x,y]], [cost,[x,y]],... ]
		if canUp(node):
			childs.append([g_child + heuristic(goal, [x, y - 1]), [x, y - 1]])
		if canDown(node):
			childs.append([g_child + heuristic(goal, [x, y + 1]), [x, y + 1]])
		if canLeft(node):
			childs.append([g_child + heuristic(goal, [x - 1, y]), [x - 1, y]])
		if canRight(node):
			childs.append([g_child + heuristic(goal, [x + 1, y]), [x + 1, y]])

		for child in childs:

			[x,y] = child[1] 
			cost_child = child[0]

			# If not explored, write its cost to frontier
			if not explored[x][y]:
				heapq.heappush(frontier, [cost_child, child[1]])
				explored[x][y] = node

			# Success condition
			if child[1] == goal:
				finish = True
				print "Succeed in finding a path via A*, writing solution to \"%s\"..." %OUTPUT_FILENAME

		if (finish):
			break


	# If finish, trace the path and write file 
	if(finish):
		[x,y] = [node[0], node[1]]
		path_cost = 1;
		solArena = copy.deepcopy(arena)

		while source != [x,y] :
			solArena[y][x] = '*'
			[prevX, prevY] = [x,y]
			[x,y] = explored[prevX][prevY]
			path_cost += 1

		writeFile(solArena, "A*: %d\n" %path_cost)
		print "Time taken:" , time.time() - start_time , "second(s)..."
		node_explored = 0
		for i in explored:
			node_explored += len(filter(None,i))

	

if results.ida:

	# Use recursive function to perform ida

	def IDA (node, g, cost_limit):

		cost = g + heuristic(node, goal)

		# If the cost exceed the limit, return that cost
		if cost > cost_limit:
			return cost
		# If succeeded, return True
		if node == goal:
			return True

		minimum = sys.maxint

		[x,y] = node

		# check if child nodes are available
		childs = []
		if canUp(node):
			childs.append([x, y - 1])
		if canDown(node):
			childs.append([x, y + 1])
		if canLeft(node):
			childs.append([x - 1, y])
		if canRight(node):
			childs.append([x + 1, y])

		for child in childs:

			# For each child node, perform IDA with g + 1
			result = IDA(child, g + 1, cost_limit)

			if (result == True):
				link.append(child)
				return True
			if result < minimum:
				minimum = result

		return minimum


	print "====================IDA===================="

	start_time = time.time()

	# "link" is used to store the path 
	link = []
	cost_limit = heuristic(source, goal)
	finish = False

	while not finish:
		result = IDA(source, 0, cost_limit)
		if result == True:
			print "Succeed in finding a path via IDA..., writing solution to \"%s\"..." %OUTPUT_FILENAME
			break
		if result == sys.maxint:
			print "Fail to find a solution via IDA..."
			break
		else:
			cost_limit = result

	# Trace the path and writing solution
	solArena = copy.deepcopy(arena)
	path_cost = len(link)
	link.pop(0)
	for node in link:
		solArena[node[1]][node[0]] = "*"

	writeFile(solArena, "IDA*: %d\n" %path_cost)
	print "Time taken:" , time.time() - start_time , "second(s)..."







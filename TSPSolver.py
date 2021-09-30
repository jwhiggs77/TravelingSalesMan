#!/usr/bin/python3

from which_pyqt import PYQT_VER
if PYQT_VER == 'PYQT5':
	from PyQt5.QtCore import QLineF, QPointF
elif PYQT_VER == 'PYQT4':
	from PyQt4.QtCore import QLineF, QPointF
else:
	raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))




import time
import numpy as np
from TSPClasses import *
from queue import PriorityQueue
import heapq
import itertools
import copy

class State:
	def __init__(self, number):
		self.n = number
		self.costMatrix = [[0 for i in range(self.n)] for j in range(self.n)]
		self.parent = None
		self.children = []
		self.cost = 0
		self.depth = 0
		self.cityID = 0
		self.path = []

	def __init__(self, number, countID):
		self.n = number
		self.costMatrix = [[0 for i in range(self.n)] for j in range(self.n)]
		self.parent = None
		self.children = []
		self.cost = 0
		self.depth = 0
		self.cityID = countID
		self.path = []

	def __gt__(self, other):
		if (self.cost > other.cost):
			return True
		else:
			return False

	def setMatrix(self, cities):
		i = 0
		j = 0
		for cityA in cities:
			for cityB in cities:
				if cityA._name == cityB._name:
					self.costMatrix[i][j] = math.inf
				else:
					self.costMatrix[i][j] = cityA.costTo(cityB)
				j += 1
			j = 0
			i += 1

	def setCost(self, newCost):
		self.cost = newCost

	def setParent(self, setParent):
		self.parent = setParent
		self.depth = setParent.depth + 1
		# self.cityID = setParent.cityID + 1
		self.cost = setParent.cost

	def addChild(self, parent):
		parent.children.append(self)

class TSPSolver:
	def __init__( self, gui_view ):
		self._scenario = None

	def setupWithScenario( self, scenario ):
		self._scenario = scenario


	''' <summary>
		This is the entry point for the default solver
		which just finds a valid random tour.  Note this could be used to find your
		initial BSSF.
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of solution, 
		time spent to find solution, number of permutations tried during search, the 
		solution found, and three null values for fields not used for this 
		algorithm</returns> 
	'''
	
	def defaultRandomTour( self, time_allowance=60.0 ):
		results = {}
		cities = self._scenario.getCities()
		ncities = len(cities)
		foundTour = False
		count = 0
		bssf = None
		start_time = time.time()
		while not foundTour and time.time()-start_time < time_allowance:
			# create a random permutation
			perm = np.random.permutation( ncities )
			route = []
			# Now build the route using the random permutation
			for i in range( ncities ):
				route.append( cities[ perm[i] ] )
			bssf = TSPSolution(route)
			count += 1
			if bssf.cost < np.inf:
				# Found a valid route
				foundTour = True
		end_time = time.time()
		results['cost'] = bssf.cost if foundTour else math.inf
		results['time'] = end_time - start_time
		results['count'] = count
		results['soln'] = bssf
		results['max'] = None
		results['total'] = None
		results['pruned'] = None
		return results


	''' <summary>
		This is the entry point for the greedy solver, which you must implement for 
		the group project (but it is probably a good idea to just do it for the branch-and
		bound project as a way to get your feet wet).  Note this could be used to find your
		initial BSSF.
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of best solution, 
		time spent to find best solution, total number of solutions found, the best
		solution found, and three null values for fields not used for this 
		algorithm</returns> 
	'''

	def greedy( self,time_allowance=60.0 ):
		random.seed(time.time())
		results = {}
		cities = self._scenario.getCities()
		foundTour = False
		count = 0  # How many tries it took to find a path
		original_cities = [cities[city] for city in range(len(cities))]  # Saves the cities for re-runs
		start_time = time.time()
		city_index = 0

		while not foundTour and time.time() - start_time < time_allowance:

			cities = [original_cities[city] for city in range(len(original_cities))]
			route = []
			# current_city = cities.pop(random.randint(0, len(cities) - 1))
			current_city = cities.pop(city_index)
			city_index += 1
			origin_city = current_city
			route.append(origin_city)

			while len(cities) > 0 and time.time() - start_time < time_allowance:

				min_distance = np.inf
				min_city = None

				# Find the minimum city
				index = -1
				for i in range(len(cities)):
					assert (type(cities[i]) == City)
					if current_city.costTo(cities[i]) < min_distance:
						min_distance = current_city.costTo(cities[i])
						min_city = cities[i]
						index = i
				cities.pop(index)  # Remove the min city from list of cities

				if min_city is not None:
					current_city = min_city

					if len(cities) == 0:
						# Check if there's a path back to the origin city
						if min_city.costTo(origin_city) < float('inf'):
							route.append(current_city)
							foundTour = True
							break
						else:
							foundTour = False
							count += 1
							break

					route.append(current_city)
				else:
					# There's no solution, rerun
					count += 1
					foundTour = False
					break

		bssf = TSPSolution(route)
		end_time = time.time()
		results['cost'] = bssf.cost if foundTour else math.inf
		results['time'] = end_time - start_time
		results['count'] = count
		results['soln'] = bssf
		results['max'] = None
		results['total'] = None
		results['pruned'] = None
		return results
	
	
	
	''' <summary>
		This is the entry point for the branch-and-bound algorithm that you will implement
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of best solution, 
		time spent to find best solution, total number solutions found during search (does
		not include the initial BSSF), the best solution found, and three more ints: 
		max queue size, total number of states created, and number of pruned states.</returns> 
	'''
		
	def branchAndBound( self, time_allowance=60.0 ):
		bestPath = []
		start_time = time.time()
		cities = self._scenario.getCities()
		nCities = len(cities)
		state1 = State(nCities, 0)
		BSSF = self.greedy(time_allowance)
		BSSF = BSSF['cost']
		# BSSF = math.inf
		lowerbound = state1.cost
		state1.setMatrix(cities)
		state1.cost = self.reduceMatrix(state1.costMatrix, nCities)
		state1.path.append(0)
		queue = PriorityQueue()
		queue.put((state1.cost, state1))

		maxQsize = 0
		totalSolutions = 0
		totalStates = 1
		numPruned = 0
		while not queue.empty() and time.time() - start_time < time_allowance:
			state = queue.get()[1]
			for i in range(nCities):
				# if there is a path
				if state.costMatrix[state.cityID][i] != math.inf:
					childState = State(nCities, i)
					childState.setParent(state)
					childState.costMatrix = copy.deepcopy(state.costMatrix)
					childState.path = copy.deepcopy(state.path)
					childState.path.append(i)
					self.setUpReduction(childState)
					childState.cost += self.reduceMatrix(childState.costMatrix, nCities)
					childState.addChild(state)
					totalStates += 1

					if childState.depth == nCities - 1:
						if childState.cost < BSSF:
							BSSF = childState.cost
							bestPath = childState.path
							totalSolutions += 1

					if childState.cost < BSSF:
						if childState.cost > lowerbound:
							queue.put((childState.cost/childState.depth, childState))
							if queue.qsize() > maxQsize:
								maxQsize = queue.qsize()
					elif childState.depth != nCities - 1:
						numPruned += 1

		finalPath = []
		for i in bestPath:
			finalPath.append(cities[i])

		results = {}
		end_time = time.time()
		results['cost'] = BSSF
		results['time'] = end_time - start_time
		results['count'] = totalSolutions
		results['soln'] = TSPSolution(finalPath)
		results['max'] = maxQsize
		results['total'] = totalStates
		results['pruned'] = numPruned
		return results

	def reduceMatrix(self, matrix, nCities):
		reduceCost = 0
		for i in range(nCities):
			lowestInRow = math.inf
			for j in range(nCities):
				if matrix[i][j] < lowestInRow:
					lowestInRow = matrix[i][j]
			if lowestInRow > 0:
				for j in range(nCities):
					if matrix[i][j] != math.inf:
						matrix[i][j] = matrix[i][j] - lowestInRow
				if lowestInRow != math.inf:
					reduceCost += lowestInRow

		for j in range(nCities):
			lowestInCol = math.inf
			for i in range(nCities):
				if matrix[i][j] < lowestInCol:
					lowestInCol = matrix[i][j]
			if lowestInCol > 0:
				for i in range(nCities):
					if matrix[i][j] != math.inf:
						matrix[i][j] = matrix[i][j] - lowestInCol
				if lowestInCol != math.inf:
					reduceCost += lowestInCol
		return reduceCost

	def setUpReduction(self, state):
		cityID = state.cityID
		parentID = state.parent.cityID
		state.cost += state.costMatrix[parentID][cityID]
		matrix = state.costMatrix

		pathLen = len(state.path)
		for i in range(pathLen):
			matrix[cityID][state.path[i]] = math.inf

		for i in range(state.n):
			matrix[i][cityID] = math.inf
		for i in range(state.n):
			matrix[parentID][i] = math.inf

	''' <summary>
		This is the entry point for the algorithm you'll write for your group project.
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of best solution, 
		time spent to find best solution, total number of solutions found during search, the 
		best solution found.  You may use the other three field however you like.
		algorithm</returns> 
	'''
		
	def fancy( self,time_allowance=60.0 ):
		pass
		




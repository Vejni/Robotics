#!/usr/bin/env python

"""
This script is the path planner. It creates the map grid, prioritises goals, and sets a trajectory using A*
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
from scipy.signal import convolve2d
from queue import PriorityQueue
from nav_msgs.msg import Path
from random import shuffle
import numpy as np
import itertools
import rospy


class Map:
	def __init__(self):
		""" Object Creator """
		self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.create_grid_callback)
		self.rate = rospy.Rate(1)
		self.grid = None
		self.costmap = None
		self.origin = None
		self.debug = rospy.get_param("/debug")

	def create_grid_callback(self, map):
		""" Callback subscribed to /map, creates grid representation that can be passed to A* """
		self.width = map.info.width
		self.height = map.info.height
		self.origin = map.info.origin.position
		self.resolution = map.info.resolution
		
		# Inflated Grid
		temp_grid = [[0 for x in range(self.width)] for y in range(self.height)]

		i = 0
		# get this param, instead of hardcode
		inflation = 6 # max inflation we can get away with
		for y in range(self.height):
			for x in range(self.width):
				if (map.data[i]!= 0):
					for yi in range(-inflation, inflation):
						for xi in range(-inflation, inflation):
							if not(y+yi < 0 or x+xi < 0 or y+yi >= self.height or x+xi >= self.width):
								temp_grid[y+yi][x+xi] = 1
				i += 1

		self.grid = temp_grid
		print("Grid created")
		self.create_costmap()
	
	def create_costmap(self):
		""" Introduces cost for going close to walls, uses blur convolution to reduce runtime """

		max_val = 10
		blur = np.ones((10, 25))

		temp = np.array([[self.grid[y][x] * max_val for x in range(self.width)] for y in range(self.height)])
		self.costmap = convolve2d(temp, blur)
		
		print("Costmap created")
		"""
		# Debug
		from matplotlib import pyplot as plt
		plt.imshow(self.costmap)
		plt.show()
		"""		

	def create_path(self, path):
		""" Convert grig path to world poses and path object """
		points = [self.get_coords(p[::-1]) for p in path]
		path = Path()
		path.header.frame_id = "map"
		for p in points:
			pose = PoseStamped()
			pose.header.frame_id = "map"
			pose.pose.position.x = p[0]
			pose.pose.position.y = p[1]
			pose.pose.position.z = p[2]
			path.poses.append(pose)

		return path
	
	def a_star(self, current_pos, current_goal, traj=False):
		""" A* with priority queue, returns the path and cost """
		print("Planning trajectory from", current_pos, "to", current_goal)
		# list is unhashable
		current_pos = (current_pos[0], current_pos[1])
		current_goal = (current_goal[0], current_goal[1])

		open_list = PriorityQueue()
		open_list.put((0, current_pos))
		
		parents = {}
		g = {}
		cost = {}
		parents[current_pos] = None
		g[current_pos] = 0

		while not open_list.empty():
			current = open_list.get()[1]
			
			# Found Goal
			if current == current_goal:
				path = []
				total_cost = 0
				while current != current_pos:
					path.append([current[0], current[1]])
					total_cost += cost[current]
					current = parents[current]
				print("Path Planning finished successfully")
				return path[::-1], total_cost

			for neighbour in self.get_neighbours(current, parents[current]):
				if traj or self.debug:
					temp = g[current] + 1
				else:
					temp = g[current] + 1 + self.costmap[neighbour[0]][neighbour[1]]
				if ((neighbour not in g) or temp < g[neighbour]):
					g[neighbour] = temp
					f = temp + self.heuristic_value(neighbour, current_goal)
					cost[neighbour] = f
					parents[neighbour] = current
					open_list.put((f, neighbour))

		print("Path Planning unsuccessful")
		return [current_pos], 0

	def heuristic_value(self, point, current_goal):
		""" Return heuristic distance from goal """
		return ((point[0] - current_goal[0])**2 + (point[1] - current_goal[1])**2)**0.5

	def set_trajectory(self, optimal=True):
		""" Find the optimal order of visiting the rooms, this function is heuristic """
		
		if optimal:
			print("Setting Trajectory (Optimal)")
			instr = rospy.get_param("/instructions")
			self.goals = [self.get_indices(goal)[::-1] for goal in instr]

			comb = list(itertools.combinations(self.goals + [self.current_position], 2))
			# cost(a, b) = cost(b, a) so remove repetition
			comb = [c for c in comb if c[::-1] not in comb]

			paths = {}
			costs = {}
			for pair in comb:
				path, cost = self.a_star(pair[0], pair[1], traj=True)
				paths[pair] = path
				costs[pair] = cost
			for g in self.goals:
				path, cost = self.a_star(self.current_position, g, traj=True)
				paths[(self.current_position, g)] = path
				costs[(self.current_position, g)] = cost

		else:
			print("Setting Trajectory (Heuristic Costs)")
			instr = rospy.get_param("/instructions")
			self.goals = [self.get_indices(goal)[::-1] for goal in instr]

		perm = list(itertools.permutations(self.goals))
		best_path = []
		min_cost = float("inf")
		for index, path in enumerate(perm):
			path = [self.current_position] + list(path)
			cost = 0
			for i in range(len(path)-1):
				if optimal:
					try:
						cost += costs[(path[i], path[i+1])]
					except:
						cost += costs[(path[i+1], path[i])]
				else:
					cost += self.heuristic_value(path[i], path[i+1])
			if cost < min_cost:
				min_cost = cost
				best_path = path

		self.trajectory = best_path
		self.current_goal = best_path[1]
		print("Trajectory found")

		total_path_it = []
		if optimal:
			for i in range(len(self.trajectory) - 1):
				try:
					path = paths[(self.trajectory[i], self.trajectory[i + 1])]
				except:
					path = paths[(self.trajectory[i + 1], self.trajectory[i])][::-1]
				total_path_it.append(path)
		else:
			for i in range(len(self.trajectory) - 1):
				lst, cost = self.a_star(self.trajectory[i], self.trajectory[i+1])
				total_path_it.append(lst)

		return total_path_it

	def set_current_position(self, data):
		""" Find and convert current position into grid representation """
		point = (data.pose.pose.position.x, data.pose.pose.position.y)
		self.current_position = self.get_indices(point)[::-1]

	def get_indices(self, point):
		""" Convert position into grid location """
		x = int((point[0] - self.origin.x) / self.resolution)
		y = int((point[1] - self.origin.y) / self.resolution)
		return (x, y)

	def get_coords(self, point):
		""" From grid location to world coordinates """
		x = point[0] * self.resolution + self.origin.x
		y = point[1] * self.resolution + self.origin.y
		return (x, y, 0)

	def get_neighbours(self, point, previous_point):
		""" Get all neighbouring nodes that are not blocked by obstacles, used by A* """
		""" Permuting the order of neighbours results in less squigly lines """

		step_size = 1
		neighbours = []
		x, y = point
		if previous_point:
			prev_x, prev_y = previous_point
		else:
			prev_x, prev_y = None, None

		for i in range(-step_size, step_size + 1, step_size):
			for j in range(-step_size, step_size + 1, step_size):
				try:
					if (
						not(self.grid[x + i][y + j]) and
						not(((x + i) == prev_x) and ((y + j) == prev_y)) and
						not((i == 0) and (j == 0))
					):
						neighbours.append((x + i, y + j))
				except IndexError:
					pass

		shuffle(neighbours)
		return neighbours


if __name__ == "__main__":
	""" For testing, this script is typically run from robot.py """
	rospy.init_node("path_planning")
	map = Map()
	while map.costmap is None:
		map.rate.sleep()
	grid = map.costmap
	path, _ = map.a_star((268, 728), (248, 773))
	for p in path:
		grid[p[0]][p[1]] = 10000
	path, _ = map.a_star((248, 773), (448, 773))
	
	for p in path:
		grid[p[0]][p[1]] = 10000

	plt.imshow(grid)
	plt.show()
	rospy.spin()

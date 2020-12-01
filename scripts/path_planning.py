#!/usr/bin/env python

"""
This script reads the map
"""

from nav_msgs.msg import OccupancyGrid
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
from queue import PriorityQueue
import rospy


class Map:
	def __init__(self):
		self.sub = rospy.Subscriber("/map", OccupancyGrid, self.create_grid_callback)
		self.rate = rospy.Rate(1)

	def create_grid_callback(self, map):
		self.width = map.info.width
		self.height = map.info.height
		self.origin = map.info.origin.position
		self.resolution = map.info.resolution

		instr = rospy.get_param("/instructions")
		self.goals = [self.get_indices(goal) for goal in instr]
		self.set_current_goal()

		"""
		y = 0
		points = []

		while y < self.height:
			for x in range(self.width):
				if map.data[y * self.width + x]:
					points.append([x, y])
			y += 1
		# Debug
		plt.scatter([p[0] for p in points], [p[1] for p in points], marker=".")
		for goal in self.goals:
			plt.scatter(goal[0], goal[1])
		plt.show()
		"""
		# Inflated Grid
		self.grid = [[0 for x in range(self.width)] for y in range(self.height)]
		i = 0
		# get this param, instead of hardcode
		inflation = 5
		for y in range(self.height):
			for x in range(self.width):
				if (map.data[i]!= 0):
					for yi in range(-inflation, inflation):
						for xi in range(-inflation, inflation):
							if not(y+yi < 0 or x+xi < 0 or y+yi >= self.height or x+xi >= self.width):
								self.grid[y+yi][x+xi] = 1
				i += 1
		plt.imshow(self.grid)
		#plt.show()
		
		self.current_goal = [5, 1]
		lst = self.a_star([2, 0], [5, 1])
		print(lst)
	
	def a_star(self, current_pos, current_goal):
		# list is unhashable
		current_pos = (current_pos[0], current_pos[1])
		current_goal = (current_goal[0], current_goal[1])

		open_list = PriorityQueue()
		open_list.put((0, current_pos))
		
		parents = {}
		g = {}
		parents[current_pos] = None
		g[current_pos] = 0

		while not open_list.empty():
			current = open_list.get()[1]
			
			# Found Goal
			if current == current_goal:
				path = []
				while current != current_pos:
					path.append([current[0], current[1]])
					current = parents[current]
				return path[::-1]

			for neighbour in self.get_neighbours(current, parents[current]):
				temp = g[current] + 1
				if ((neighbour not in g) or temp < g[neighbour]):
					g[neighbour] = temp
					f = temp + self.heuristic_value(neighbour)
					parents[neighbour] = current
					open_list.put((f, neighbour))

		return [current_pos]

	def heuristic_value(self, point):
		return ((point[0] - self.current_goal[0])**2 + (point[1] - self.current_goal[1])**2)**0.5

	def set_current_goal(self):
		# TODO
		self.current_goal = self.goals[0]

	def get_indices(self, point):
		x = int((point[0] - self.origin.x) / self.resolution)
		y = int((point[1] - self.origin.y) / self.resolution)
		return (x, y)

	def get_neighbours(self, point, previous_point):
		# TODO: Refactor

		step_size = 1
		neighbours = []
		x, y = point
		if previous_point:
			prev_x, prev_y = previous_point
		else:
			prev_x, prev_y = None, None


		# up
		try:
			if (not(self.grid[x][y + step_size]) and not(x == prev_x and y == prev_y)):
				neighbours.append((x, y + step_size))
		except IndexError:
			pass
		# down
		try:
			if (not(self.grid[x][y - step_size]) and not(x == prev_x and y == prev_y)):
				neighbours.append((x, y - step_size))
		except IndexError:
			pass
		# left
		try:
			if (not(self.grid[x - step_size][y]) and not(x == prev_x and y == prev_y)):
				neighbours.append((x - step_size, y))
		except IndexError:
			pass
		# right
		try:
			if (not(self.grid[x + step_size][y]) and not(x == prev_x and y == prev_y)):
				neighbours.append((x + step_size, y))
		except IndexError:
			pass

		print(point, ":", neighbours)
		return neighbours

	def create_voronoi_callback(self, map):
		""" Old """
		"""
		# Inflated Grid
		self.grid = [[0 for x in range(self.width)] for y in range(self.height)]
		i = 0
		# get this param, instead of hardcode
		inflation = 5
		for y in range(self.height):
			for x in range(self.width):
				if (map.data[i]!= 0):
					for yi in range(-inflation, inflation):
						for xi in range(-inflation, inflation):
							if not(y+yi < 0 or x+xi < 0 or y+yi >= self.height or x+xi >= self.width):
								self.grid[y+yi][x+xi] = 1
				i += 1
		"""
		self.width = map.info.width
		self.height = map.info.height
		y = 0
		points = []

		while y < self.height:
			for x in range(self.width):
				if map.data[y * self.width + x]:
					points.append([x, y])
			y += 1

		vor = Voronoi(points)
		tree = cKDTree(vor.vertices)
		data = [p for p in tree.data if (p[0] >= 0 and p[1] <= 500 and p[1] >= 0)]
		dist, indices = tree.query(vor.vertices[0], k=100, distance_upper_bound=100)
		
		# Debug
		plt.scatter([p[0] for p in points], [p[1] for p in points], marker=".")
		plt.scatter([p[0] for p in data], [p[1] for p in data], marker=".")
		plt.scatter([p[0] for p in vor.vertices[indices]], [p[1] for p in vor.vertices[indices]], marker=".")		
		plt.show()
		

if __name__ == "__main__":
	rospy.init_node("path_planning")
	Map()
	rospy.spin()

#!/usr/bin/env python

"""
This script reads the map
"""

from nav_msgs.msg import OccupancyGrid
from matplotlib import pyplot as plt
from scipy.spatial import Voronoi, cKDTree
from json import dump
import rospy


class Map:
	def __init__(self):
		rospy.init_node("path_planning")
		self.sub = rospy.Subscriber("/map", OccupancyGrid, self.create_grid_callback)
		self.rate = rospy.Rate(1)
		

	def create_grid_callback(self, map):
		self.width = map.info.width
		self.height = map.info.height
		self.origin = map.info.origin.position
		self.resolution = map.info.resolution

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
		y = 0
		points = []

		while y < self.height:
			for x in range(self.width):
				if map.data[y * self.width + x]:
					points.append([x, y])
			y += 1

		instr = rospy.get_param("/instructions")
		goals = [self.get_indices(goal) for goal in instr]

		# Debug
		plt.scatter([p[0] for p in points], [p[1] for p in points], marker=".")
		for goal in goals:
			plt.scatter(goal[0], goal[1])
		plt.show()

	def get_indices(self, point):
		x = int((point[0] - self.origin.x) / self.resolution)
		y = int((point[1] - self.origin.y) / self.resolution)
		return (x, y)
	
	def create_voronoi_callback(self, map):
		
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
	Map()
	rospy.spin()

#!/usr/bin/env python

"""
This script reads the map
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
from queue import PriorityQueue
from nav_msgs.msg import Path
import rospy


class Map:
	def __init__(self):
		self.sub = rospy.Subscriber("/map", OccupancyGrid, self.create_grid_callback)
		self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
		self.rate = rospy.Rate(1)

	def create_grid_callback(self, map):
		self.width = map.info.width
		self.height = map.info.height
		self.origin = map.info.origin.position
		self.resolution = map.info.resolution

		instr = rospy.get_param("/instructions")
		self.goals = [self.get_indices(goal) for goal in instr]
		self.set_current_goal()
		self.set_current_position()
		
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
		
		lst = self.a_star(self.current_position, self.current_goal)
		"""
		# Debug
		self.grid[self.current_position[0]][self.current_position[1]] = 1
		self.grid[self.current_goal[0]][self.current_goal[1]] = 1
		for p in lst:
			self.grid[p[0]][p[1]] = 1
		plt.imshow(self.grid)
		plt.show()
		"""
		"""
		# Debug
		y = 0
		points = []

		while y < self.height:
			for x in range(self.width):
				if map.data[y * self.width + x]:
					points.append([x, y])
			y += 1

		plt.scatter([p[0] for p in points], [p[1] for p in points], marker=".")
		plt.scatter(self.current_goal[0], self.current_goal[1])
		plt.scatter(self.current_position[0], self.current_position[1], marker=",")
		plt.scatter([p[0] for p in lst], [p[1] for p in lst], marker=".")
		plt.show()
		"""

		self.publish_path(lst)

	def publish_path(self, path):
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
		print(path)
		while not rospy.is_shutdown():
			self.path_pub.publish(path)
			self.rate.sleep()
	
	def a_star(self, current_pos, current_goal):
		print("Planning trajectory from", current_pos, "to", current_goal)
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
		self.current_goal = self.goals[0][::-1]

	def set_current_position(self):
		# TODO 
		point = (2, 0)
		self.current_position = self.get_indices(point)[::-1]

	def get_indices(self, point):
		x = int((point[0] - self.origin.x) / self.resolution)
		y = int((point[1] - self.origin.y) / self.resolution)
		return (x, y)

	def get_coords(self, point):
		x = point[0] * self.resolution + self.origin.x
		y = point[1] * self.resolution + self.origin.y
		return (x, y, 0)

	def get_neighbours(self, point, previous_point):
		# TODO: Refactor

		step_size = 1
		neighbours = []
		x, y = point
		if previous_point:
			prev_x, prev_y = previous_point
		else:
			prev_x, prev_y = None, None

		# down
		try:
			# print(self.grid[x][y + step_size])
			if (not(self.grid[x][y + step_size]) and not(x == prev_x and y + step_size == prev_y)):
				neighbours.append((x, y + step_size))
		except IndexError:
			pass
		# up
		try:
			# print(self.grid[x][y - step_size])
			if (not(self.grid[x][y - step_size]) and not(x == prev_x and y - step_size == prev_y)):
				neighbours.append((x, y - step_size))
		except IndexError:
			pass
		# left
		try:
			# print(self.grid[x - step_size][y])
			if (not(self.grid[x - step_size][y]) and not(x - step_size == prev_x and y == prev_y)):
				neighbours.append((x - step_size, y))
		except IndexError:
			pass
		# right
		try:
			# print(self.grid[x + step_size][y])
			if (not(self.grid[x + step_size][y]) and not(x + step_size == prev_x and y == prev_y)):
				neighbours.append((x + step_size, y))
		except IndexError:
			pass

		print(point, ":", neighbours)
		return neighbours


if __name__ == "__main__":
	rospy.init_node("path_planning")
	Map()
	rospy.spin()

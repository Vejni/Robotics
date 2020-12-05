#!/usr/bin/env python

from nav_msgs.msg import Odometry, Path
from read_config import read_config
from controller import Controller
from path_planner import Map
import itertools
import rospy

class Robot:
	def __init__(self):
		rospy.init_node("Robot")
		read_config()
		self.rate = rospy.Rate(1)
		self.rate.sleep()

		self.map = Map()
		self.map.grid = None
		self.pos_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.set_current_position)
		self.total_path_pub = rospy.Publisher("/total_planned_path", Path, queue_size=1)
		self.current_path_pub = rospy.Publisher("/current_planned_path", Path, queue_size=1)

		while self.map.grid is None:
			self.rate.sleep()
		self.map.set_trajectory()
		
		total_path_it = []
		for i in range(1,len(self.map.trajectory) - 1):
			lst, cost = self.map.a_star(self.map.trajectory[i], self.map.trajectory[i+1])
			total_path_it.append(lst)
		
		total_path = self.map.create_path(list(itertools.chain.from_iterable(total_path_it)))

		contr = Controller()
		contr.path = self.map.create_path(total_path_it[0])
		while contr.position is None:
			self.rate.sleep()

		goal_index = 0
		while not rospy.is_shutdown():
			if not contr.arrived:
				contr.set_next_goal()
				contr.drive()
				self.current_path_pub.publish(contr.path)
			else:
				print("Destination reached")
				goal_index += 1
				if goal_index < len(total_path_it):
					contr.arrived = False
					contr.path = self.map.create_path(total_path_it[goal_index])
			self.total_path_pub.publish(total_path)

	def set_current_position(self, data):
		self.map.set_current_position(data)
		

if __name__ == "__main__":
	try:
		rob = Robot()
		rospy.spin()
	except Exception as e:
		print(e)

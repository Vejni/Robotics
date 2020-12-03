#!/usr/bin/env python

from nav_msgs.msg import Odometry, Path
from read_config import read_config
from path_planner import Map
from controller import Controller
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
		
		total_path = []
		for i in range(1,len(self.map.trajectory) - 1):
			lst, cost = self.map.a_star(self.map.trajectory[i], self.map.trajectory[i+1])
			total_path += lst

		current_path, cost = self.map.a_star(self.map.current_position, self.map.current_goal)
		
		total_path = self.map.create_path(total_path)
		current_path = self.map.create_path(current_path)

		contr = Controller()
		contr.path = current_path
		while contr.position is None:
			self.rate.sleep()

		while not rospy.is_shutdown():
			if contr.path:
				contr.set_next_goal()
				contr.drive()
				self.current_path_pub.publish(current_path)
			self.total_path_pub.publish(total_path)

			self.rate.sleep()

	def set_current_position(self, data):
		self.map.set_current_position(data)
		

if __name__ == "__main__":
	try:
		rob = Robot()
		rospy.spin()
	except Exception as e:
		print(e)

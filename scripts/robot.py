#!/usr/bin/env python

from nav_msgs.msg import Odometry, Path
from path_planner import Map
from read_config import read_config
import rospy

class Robot:
	def __init__(self):
		rospy.init_node("Robot")
		read_config()
		self.rate = rospy.Rate(1)
		self.rate.sleep()

		self.map = Map()
		self.pos_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.set_current_position)
		self.total_path_pub = rospy.Publisher("/total_planned_path", Path, queue_size=1)
		self.current_path_pub = rospy.Publisher("/current_planned_path", Path, queue_size=1)

		self.rate.sleep()
		self.rate.sleep()
		rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=None)
		self.map.set_trajectory()
		
		total_path = []
		for i in range(1,len(self.map.trajectory) - 1):
			lst, cost = self.map.a_star(self.map.trajectory[i], self.map.trajectory[i+1])
			total_path += lst

		current_path, cost = self.map.a_star(self.map.current_position, self.map.current_goal)
		
		total_path = self.map.create_path(total_path)
		current_path = self.map.create_path(current_path)
		while not rospy.is_shutdown():
			try:
				self.total_path_pub.publish(total_path)
				self.current_path_pub.publish(current_path)
			except Exception as e:
				print(e)

	def set_current_position(self, data):
		self.map.set_current_position(data)
		

if __name__ == "__main__":
	try:
		rob = Robot()
		rospy.spin()
	except Exception as e:
		print(e)

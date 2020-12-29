#!/usr/bin/env python

from nav_msgs.msg import Odometry, Path
from read_config import read_config
from controller import Controller
from std_msgs.msg import Bool
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
		self.pos_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.set_current_position)
		self.total_path_pub = rospy.Publisher("/total_planned_path", Path, queue_size=1)
		self.current_path_pub = rospy.Publisher("/current_planned_path", Path, queue_size=1)
		self.vaccum_pub = rospy.Publisher("/vacuum", Bool, queue_size=1)

		while self.map.costmap is None:
			self.rate.sleep()

		total_path_it = self.map.set_trajectory(optimal=False)
		total_path = self.map.create_path(list(itertools.chain.from_iterable(total_path_it)))

		contr = Controller()
		contr.path = self.map.create_path(total_path_it[0])
		while contr.position is None:
			self.rate.sleep()

		goal_index = 0
		finished = False
		contr.vacuuming = False
		while not rospy.is_shutdown() or not finished:
			if not contr.arrived and not contr.vacuuming:
				contr.set_next_goal()
				contr.drive()
				self.current_path_pub.publish(contr.path)

			elif contr.vacuuming:
				contr.vacuuming = False
				contr.vacuum()
				self.vaccum_pub.publish(True)

			else:
				print("Destination reached - Starting to vacuum")
				contr.vacuuming = True
				goal_index += 1
				if goal_index < len(total_path_it):
					contr.arrived = False
					contr.path = self.map.create_path(total_path_it[goal_index])
				else:
					finished = True

			self.total_path_pub.publish(total_path)

	def set_current_position(self, data):
		self.map.set_current_position(data)
		

if __name__ == "__main__":
	try:
		rob = Robot()
		rospy.spin()
	except Exception as e:
		print(e)

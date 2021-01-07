#!/usr/bin/env python

""" Main script of the project """

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32
from read_config import read_config
from controller import Controller
from path_planner import Map
import itertools
import rospy

class Robot:
	def __init__(self):
		rospy.init_node("Robot")
		self.rate = rospy.Rate(1)
		self.map = Map()
		self.battery_low = False
		self.no_vacuum = rospy.get_param("/no_vacuum")

		# Subscribers
		self.pos_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.set_current_position)
		self.battery_sub = rospy.Subscriber("/battery", Float32, self.battery_check)

		# Publishers
		self.current_path_pub = rospy.Publisher("/current_planned_path", Path, queue_size=1)
		self.total_path_pub = rospy.Publisher("/total_planned_path", Path, queue_size=1)
		self.vaccum_pub = rospy.Publisher("/vacuum", Bool, queue_size=1)

		# Wait for map
		while self.map.costmap or self.map.origin is None:
			self.rate.sleep()

		# Give it some more time
		rospy.Rate(0.2).sleep()

		# Set trajectory
		total_path_it = self.map.set_trajectory(optimal=rospy.get_param("optimal_path"))
		total_path = self.map.create_path(list(itertools.chain.from_iterable(total_path_it)))

		# Set up the controller
		self.contr = Controller()
		self.contr.path = self.map.create_path(total_path_it[0])
		while self.contr.position is None:
			self.rate.sleep()

		# Init
		goal_index = 0
		finished = False
		self.contr.vacuuming = False
		self.recharged = False

		# Main loop
		while not rospy.is_shutdown() or not finished:
			if not self.contr.arrived and not self.contr.vacuuming and not self.battery_low:
				self.follow_path()

			elif self.contr.vacuuming and not self.battery_low:
				self.vacuum()
			
			elif self.battery_low:
				self.follow_path()

			else:
				print("Destination reached - Starting to vacuum")
				if self.recharged:
					contr.path = self.save_path
					if self.stopped_vacuuming:
						self.battery_low = False
						if not self.no_vacuum:						
							self.contr.vacuuming = True
				else:
					if not self.no_vacuum:
						self.contr.turn_to_angle(0)
						while self.contr.turning:
							self.rate.sleep()

						self.contr.read_vacuum = True
						self.contr.vacuuming = True

					goal_index += 1
					if goal_index < len(total_path_it):
						self.contr.arrived = False
						self.contr.path = self.map.create_path(total_path_it[goal_index])
					else:
						finished = True

			self.total_path_pub.publish(total_path)

	def follow_path(self):
		self.contr.set_next_goal()
		self.contr.drive()
		self.current_path_pub.publish(self.contr.path)
		self.vaccum_pub.publish(False)

	def vacuum(self):
		while self.contr.read_vacuum:
			self.rate.sleep()

		self.contr.set_next_goal()
		self.contr.drive()
		self.vaccum_pub.publish(True)
		self.current_path_pub.publish(self.contr.vacuum_path)

	def set_current_position(self, data):
		self.map.set_current_position(data)

	def battery_check(self, data):
		threshold = 40
		if int(data.data) % 5 == 0:
			print("Battery at: ", int(data.data))
		if data.data <= threshold:
			if not self.battery_low:
				print("Battery low")
				self.battery_low = True
				if self.contr.vacuuming and self.contr.vacuum_path:
					self.save_path = self.contr.vacuum_path
					self.stopped_vacuuming = True
				else:
					self.save_path = self.contr.path
				self.contr.path = self.map.plan_charging()
				self.contr.vacuuming = False

		if data.data > 98 and self.battery_low:
			self.contr.path = self.map.replan(self.contr.path)
			self.recharged = True

if __name__ == "__main__":
	try:
		read_config()
		rob = Robot()
		rospy.spin()
	except Exception as e:
		print(e)

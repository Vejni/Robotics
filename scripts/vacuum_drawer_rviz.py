#!/usr/bin/env python

"""
This scripts reads the vacuuming radius and leaves markers indicating vacuuming progress in rviz
"""

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import rospy


class VacuumMarkers:
	def __init__(self, topic, id, r):
		self.vacuuming = False
		self.pub = rospy.Publisher(topic, MarkerArray, queue_size=1)
		self.sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.add)
		self.vacuum_sub = rospy.Subscriber("/vacuum", Bool, self.switch)
		self.id = id
		self.rate = rospy.Rate(200)
		self.rate.sleep()
		self.MarkerArray = MarkerArray()
		self.radius = r

	def switch(self, vacuuming):
		self.vacuuming = vacuuming
		if not self.vacuuming:
			self.clean_markers()
		self.rate.sleep()

	def add(self, goal):
		if self.vacuuming:
			self.goal = Marker()
			self.goal.type = 3
			self.goal.action = Marker.ADD
			self.goal.header.frame_id="map"
			self.id += 1
			self.goal.id = self.id
			self.goal.lifetime = rospy.Duration(0)

			# Colour
			self.goal.color.r = 0.5
			self.goal.color.b = 1
			self.goal.color.g = 0
			self.goal.color.a = 0.7
			
			# Size
			self.goal.scale.x = self.radius
			self.goal.scale.y = self.radius
			self.goal.scale.z = 0.001

			# Position
			self.goal.pose.position.x = goal.pose.pose.position.x
			self.goal.pose.position.y = goal.pose.pose.position.y
			self.goal.pose.position.z = 0

			# Orientation
			self.goal.pose.orientation.x = 0.0
			self.goal.pose.orientation.y = 0.0
			self.goal.pose.orientation.z = 0.0
			self.goal.pose.orientation.w = 1.0

			self.MarkerArray.markers.append(self.goal)
			self.send()
		
	def send(self):
		self.pub.publish(self.MarkerArray)
		self.rate.sleep()

	def clean_markers(self):
		self.MarkerArray = MarkerArray()
		if self.MarkerArray.markers:
			for m in self.MarkerArray.markers:
				m.action = m.DELETE
				self.MarkerArray.markers.append(m)
		self.pub.publish(self.MarkerArray)
     

if __name__ == "__main__":
	rospy.init_node('vacuum_markers')
	radius = rospy.get_param("/vacuum_radius")

	goal_markers = VacuumMarkers("/vacuum_markers", 1, radius)

	rospy.spin()

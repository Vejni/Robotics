#!/usr/bin/env python

"""
This scripts reads the instructions parameter and creates markers for the goals and chargers in rviz
"""

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
import rospy


class Markers:
	def __init__(self, topic, id):
		
		self.pub = rospy.Publisher(topic, MarkerArray, queue_size=200)
		self.id = id
		rospy.sleep(1)
		self.MarkerArray = MarkerArray()
	
	def add(self, goal, shape, rgb):
		
		self.goal = Marker()
		self.goal.type = shape
		self.goal.action = Marker.ADD
		self.goal.header.frame_id="map"
		self.id += 1
		self.goal.id = self.id
		self.goal.lifetime = rospy.Duration(0)

		# Colour
		self.goal.color.r = rgb[0]
		self.goal.color.g = rgb[1]
		self.goal.color.b = rgb[2]
		self.goal.color.a = 1
		
		# Size
		self.goal.scale.x = 0.3
		self.goal.scale.y = 0.3
		self.goal.scale.z = 0.3

		# Position
		self.goal.pose.position.x = goal[0]
		self.goal.pose.position.y = goal[1]
		self.goal.pose.position.z = 0.0

		# Orientation
		self.goal.pose.orientation.x = 0.0
		self.goal.pose.orientation.y = 0.0
		self.goal.pose.orientation.z = 0.0
		self.goal.pose.orientation.w = 1.0

		self.MarkerArray.markers.append(self.goal)
		
	def send(self):
		self.pub.publish(self.MarkerArray)

	def clean_markers(self):
		self.MarkerArray = MarkerArray()
		if self.MarkerArray.markers:
			for m in elf.MarkerArray.markers:
				m.action = m.DELETE
				self.MarkerArray.markers.append(m)
		self.pub.publish(self.MarkerArray)
     

if __name__ == "__main__":
	rospy.init_node('simple_markers')
	markers = Markers("/goals", 0)

	# Goals
	instr = rospy.get_param("/instructions")
	for goal in instr:
		markers.add(goal, Marker.SPHERE, (0,0,1))

	# Charging points
	chargers = rospy.get_param("/chargers")
	for c in chargers:
		markers.add(c, Marker.CUBE, (1, 1, 0))


	markers.send()

	rospy.spin()

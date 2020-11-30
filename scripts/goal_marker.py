#!/usr/bin/env python

"""
This scripts reads the instructions parameter and creates markers for the goals in rviz
"""

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion


class goalMarkers:
	def __init__(self):
		
		self.pub=self.pub=rospy.Publisher("/goals", MarkerArray, queue_size=200)
		self.id = 0
		rospy.sleep(1)
		self.MarkerArray=MarkerArray()

	
	def add(self, goal):
		
		self.goal = Marker()
		self.goal.type = Marker.SPHERE
		self.goal.action=Marker.ADD
		self.goal.header.frame_id="map"
		self.id += 1
		self.goal.id = self.id
		self.goal.lifetime = rospy.Duration(0)

		# Colour
		self.goal.color.r=0
		self.goal.color.b=1
		self.goal.color.g=0
		self.goal.color.a=1
		
		# Size
		self.goal.scale.x=0.3
		self.goal.scale.y=0.3
		self.goal.scale.z=0.3

		# Position
		self.goal.pose.position.x = goal[0]
		self.goal.pose.position.y = goal[1]
		self.goal.pose.position.z = 0

		self.MarkerArray.markers.append(self.goal)
		
	def send(self):
		self.pub.publish(self.MarkerArray)
     

if __name__ == "__main__":
	rospy.init_node('goal_markers')
	instr = rospy.get_param("/instructions")
	print(instr)

	goal_markers = goalMarkers()
	for goal in instr:
		goal_markers.add(goal)
		print(goal)
	goal_markers.send()

	rospy.spin()

#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from goal_marker import goalMarkers
import rospy
import math

class Controller:
	def __init__(self):
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 1)
		rospy.wait_for_message("/base_pose_ground_truth", Odometry)
		self.pose_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.get_pose)
		self.cmd_vel = Twist()
		self.epsilon = 0.05
		self.position = None
		self.rate = rospy.Rate(100)
		self.markers = goalMarkers("/debug_goal", 1)
		self.arrived = False
	
	def get_pose(self, data):
		self.position = data.pose.pose.position
		quat = data.pose.pose.orientation
		_, _, self.theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

	def drive(self):
		if not self.arrived:
			dx = abs(self.position.x - self.goal.x)
			dy = abs(self.position.y - self.goal.y)

			if(dx > self.epsilon or dy > self.epsilon):
				
				# First turn
				while(abs(self.theta - self.goal_angle) > math.pi/18):
					self.cmd_vel.angular.z = self.limit_rotation((self.goal_angle - self.theta))
					self.cmd_vel.linear.x = 0.0
					self.pub.publish(self.cmd_vel)

				# Forward
				self.cmd_vel.angular.z = 0.0
				self.cmd_vel.linear.x = self.sigmoid(dx+dy) * 2
				self.pub.publish(self.cmd_vel)

				#rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=None)
				dx = abs(self.position.x - self.goal.x)
				dy = abs(self.position.y - self.goal.y)
				self.rate.sleep()

			print("I moved")

	def set_next_goal(self):
		try:
			self.goal = self.path.poses.pop(0).pose.position
			self.goal_angle = math.atan2(self.goal.y - self.position.y, self.goal.x - self.position.x)
		except:
			self.arrived = True
	
	def sigmoid(self, x):
		return 1 / (1 + math.exp(-(10 * (x - 0.3))))

	def limit_rotation(self, angle):
		while angle <= math.pi:
			angle += 2*math.pi
		while angle > math.pi:
			angle -= 2*math.pi
		return angle

	

if __name__ == "__main__":
	rospy.init_node("controller")
	r = rospy.Rate(3)
	contr = Controller()
	rospy.spin()

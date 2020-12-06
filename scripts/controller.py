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
		self.prev_position = None
		self.stuck = False
		self.turn_speed = 1.5
		self.forward_speed = 5

		self.look_ahead = 0.05
		self.pursuit_intersect = None
	
	def get_pose(self, data):
		self.position = data.pose.pose.position
		quat = data.pose.pose.orientation
		_, _, self.theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

	def drive(self):
		if not self.arrived:
			dx = abs(self.position.x - self.goal.x)
			dy = abs(self.position.y - self.goal.y)

			if(dx > self.epsilon or dy > self.epsilon):
				
				# Turn
				while(abs(self.theta - self.goal_angle) > math.pi/6):
					self.cmd_vel.angular.z = self.limit_rotation((self.goal_angle - self.theta)) * self.turn_speed
					self.cmd_vel.linear.x = 0.0
					self.pub.publish(self.cmd_vel)

				while(abs(self.theta - self.goal_angle) > math.pi/15):
					self.cmd_vel.angular.z = self.limit_rotation((self.goal_angle - self.theta)) * self.turn_speed
					self.cmd_vel.linear.x = 0.1
					self.pub.publish(self.cmd_vel)

				# Forward
				self.cmd_vel.angular.z = 0.0
				self.cmd_vel.linear.x = self.sigmoid(dx+dy) * self.forward_speed
				self.pub.publish(self.cmd_vel)

				# Stuck?
				""" # TODO
				if self.prev_position == self.position:
					if self.stuck:
						self.cmd_vel.linear.x = -0.25
						self.pub.publish(self.cmd_vel)
						self.stuck = False
					else:
						self.stuck = True
				self.prev_position = self.position
				"""				

				#rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=None)
				dx = abs(self.position.x - self.goal.x)
				dy = abs(self.position.y - self.goal.y)

				self.rate.sleep()

			print("I moved")

	def set_next_goal_pursuit(self):
		if self.pursuit_intersect is not None:
			current_index = self.pursuit_intersect
		
			d_current = ((self.position.x - self.path.poses[current_index].pose.position.x)**2 + (self.position.y - self.path.poses[current_index].pose.position.y)**2)**0.5
			d_next = float("inf")
			while d_current < d_next:
				if current_index < len(self.path.poses) - 1:
					current_index += 1
				d_current = d_next
				d_next = ((self.position.x - self.path.poses[current_index].pose.position.x)**2 + (self.position.y - self.path.poses[current_index].pose.position.y)**2)**0.5
			self.pursuit_intersect = current_index		
		else:
			min_d = float("inf")
			for i, p in enumerate(self.path.poses):
				d = (self.position.x - p.pose.position.x)**2 + (self.position.y - p.pose.position.y)**2
				if d < min_d:
					min_d = d
					self.pursuit_intersect = i
		
		while (
			self.look_ahead > ((self.position.x - self.path.poses[self.pursuit_intersect].pose.position.x)**2 + (self.position.y - self.path.poses[self.pursuit_intersect].pose.position.y)**2)**0.5 and
			self.pursuit_intersect < len(self.path.poses)
		):
			self.pursuit_intersect += 1


	def pure_pursuit(self):
		if not self.arrived:
			self.set_next_goal_pursuit()
			print(self.pursuit_intersect)
			if self.pursuit_intersect < len(self.path.poses):
				px = self.path.poses[self.pursuit_intersect].pose.position.x
				py = self.path.poses[self.pursuit_intersect].pose.position.y
			else:
				px = self.path.poses[-1].pose.position.x
				py = self.path.poses[-1].pose.position.y

			alpha = math.atan2(py - self.position.y, px - self.position.x) - self.theta
			delta = 2*math.sin(alpha) / self.look_ahead
			self.cmd_vel.angular.z = delta
			self.cmd_vel.linear.x = (((py -self.position.y)**2 + (px - self.position.x)**2)**0.5) * 1.25
			"""
			r = (self.look_ahead ** 2) / (2 * (px - self.position.x))
			self.cmd_vel.linear.x = self.sigmoid(py) * 0.2
			self.cmd_vel.angular.z = self.limit_rotation(self.cmd_vel.linear.x / r)
			"""

			print("p:", px, py)
			print(self.position.x, self.position.y)
			print(self.cmd_vel.linear.x, self.cmd_vel.angular.z)
			self.pub.publish(self.cmd_vel)

			self.look_ahead = self.cmd_vel.linear.x * 0.2
			self.rate.sleep()


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

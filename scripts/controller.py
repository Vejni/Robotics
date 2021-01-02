#!/usr/bin/env python

""" This is the main controller, plus the scanner logic for obstacle avoidance """

from geometry_msgs.msg import Point, Twist, PointStamped, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import rospy
import math

class Controller:
	def __init__(self):
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 1)
		rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
		self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.get_pose)
		self.laser_sub = rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
		self.following = False
		self.cmd_vel = Twist()
		self.epsilon = 0.03
		self.collision_epsilon = 0.03
		self.position = None
		self.rate = rospy.Rate(100)

		self.arrived = False
		self.prev_position = None
		self.stuck = False
		self.turn_speed = 2
		self.forward_speed = 3
		self.robot_width = 3
		self.speed = 0
		self.counter = 0
		self.read_vacuum = False

	def laser_callback(self, data):
		regions = {
			'right':  min(min(data.ranges[0:100]), 10),
			'fright': min(min(data.ranges[101:499]), 10),
			'front':  min(min(data.ranges[500:600]), 10),
			'fleft':  min(min(data.ranges[601:980]), 10),
			'left':   min(min(data.ranges[981:1081]), 10),
		}

		# Obstacle in front
		if regions['front'] < self.collision_epsilon:
			print("Obstacle too close!")
			self.stuck = True
			if regions['left'] < self.collision_epsilon:
				self.dir = 1
			elif regions['right'] < self.collision_epsilon:
				self.dir = -1
			else:
				self.dir = 0

			self.cmd_vel.angular.z = 0.4 * self.dir
			self.cmd_vel.linear.x = -0.5
			self.pub.publish(self.cmd_vel)

		# next to wall on the left, don't turn
		elif regions['left'] < self.collision_epsilon:
			print("Next to wall, left")
			self.following = True
			
			# First turn parallel to obstacle
			while regions['fleft'] < self.collision_epsilon:
				self.cmd_vel.angular.z = -0.3
				self.cmd_vel.linear.x = 0
				self.pub.publish(self.cmd_vel)

			# Then go ahead
			self.cmd_vel.linear.x = 0.5
			self.cmd_vel.angular.z = 0
			self.pub.publish(self.cmd_vel)

		# next to wall on the right, don't turn
		elif regions['right'] < self.collision_epsilon:
			print("Next to wall, right")
			self.following = True
			
			# First turn parallel to obstacle
			while regions['fright'] < self.collision_epsilon:
				self.cmd_vel.angular.z = 0.3
				self.cmd_vel.linear.x = 0
				self.pub.publish(self.cmd_vel)

			# Then go ahead
			self.cmd_vel.linear.x = 0.5
			self.cmd_vel.angular.z = 0
			self.pub.publish(self.cmd_vel)

		else:
			self.stuck = False
			self.following = False

		if self.read_vacuum:
			self.read_room(regions)

		self.rate.sleep()

	def read_room(self, regions):
		# Get distance to walls in each directions
		d_west = regions["front"]
		d_south = regions["right"]
		d_north = regions["left"]

		# No scanner on the back, so turn around
		self.turn_to_angle(math.pi)
		while self.turning:
			self.rate.sleep()
		d_east = regions["front"]
		d_robot = 0.3

		p_north_east = Point()
		p_north_east.x = self.position.x - d_east + d_robot
		p_north_east.y = self.position.y + d_north - d_robot
		p_north_east.z = 0

		p_north_west = Point()
		p_north_west.x = self.position.x + d_west - d_robot
		p_north_west.y = self.position.y + d_north - d_robot
		p_north_west.z = 0

		p_south_east = Point()
		p_south_east.x = self.position.x - d_east + d_robot
		p_south_east.y = self.position.y - d_south + d_robot
		p_south_east.z = 0

		p_south_west = Point()
		p_south_west.x = self.position.x + d_west - d_robot
		p_south_west.y = self.position.y - d_south + d_robot
		p_south_west.z = 0

		self.room_points = [p_north_east, p_north_west, p_south_east, p_south_west]
		self.create_vacuum_path()

		# Flip the switch
		self.read_vacuum = False

	def create_vacuum_path(self):
		path = Path()
		path.header.frame_id = "map"
		even = 1
		for x in np.linspace(self.room_points[0].x, self.room_points[1].x + 0.15, 15):
			if even == 1:
				start = self.room_points[1].y
				end = self.room_points[2].y
			else:
				end = self.room_points[1].y
				start = self.room_points[2].y		

			for y in np.linspace(start, end, 50):
				pose = PoseStamped()
				pose.header.frame_id = "map"
				pose.pose.position.x = x
				pose.pose.position.y = y
				pose.pose.position.z = 0
				path.poses.append(pose)
			even *= -1

		self.vacuum_path = path
		
	def turn_to_angle(self, angle):
		self.turning = True
		while abs(self.theta - angle) > math.pi/18:
			self.cmd_vel.linear.x = 0
			self.cmd_vel.angular.z = 0.5
			self.pub.publish(self.cmd_vel)
			self.rate.sleep()

		self.cmd_vel.linear.x = 0
		self.cmd_vel.angular.z = 0
		self.pub.publish(self.cmd_vel)

		self.turning = False

	def get_pose(self, data):
		self.position = data.pose.pose.position
		quat = data.pose.pose.orientation
		_, _, self.theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

	def drive(self):
		if not self.arrived and not self.stuck and not self.following:
			dx = abs(self.position.x - self.goal.x)
			dy = abs(self.position.y - self.goal.y)

			if(dx > self.epsilon or dy > self.epsilon):
				
				# Turn
				while(abs(self.theta - self.goal_angle) > math.pi/9):
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

				#rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=None)
				dx = abs(self.position.x - self.goal.x)
				dy = abs(self.position.y - self.goal.y)

				self.rate.sleep()

	def set_next_goal(self):
		try:	
			if self.vacuuming:
				self.goal = self.vacuum_path.poses.pop(0).pose.position
			else:
				self.goal = self.path.poses.pop(0).pose.position
			self.goal_angle = math.atan2(self.goal.y - self.position.y, self.goal.x - self.position.x)
		except:
			if self.vacuuming:
				self.vacuuming = False
			else:
				self.arrived = True
	
	def sigmoid(self, x):
		return 1 / (1 + math.exp(-(10 * (x - 0.3))))

	def limit_rotation(self, angle):
		while angle <= math.pi:
			angle += 2*math.pi
		while angle > math.pi:
			angle -= 2*math.pi
		return angle
	
	def vacuum(self):
		if not self.following and not self.stuck:
			self.cmd_vel.angular.z = 1
			if self.speed <= 0.8:
				self.cmd_vel.linear.x = self.speed + 0.001
				self.speed = self.cmd_vel.linear.x
			elif self.counter <= 100:
				self.cmd_vel.linear.x = self.speed
				self.counter += 1
			else:
				self.counter = 0
				self.vacuuming = False
			self.pub.publish(self.cmd_vel)
		self.rate.sleep()


if __name__ == "__main__":
	rospy.init_node("controller")
	r = rospy.Rate(3)
	contr = Controller()
	rospy.spin()

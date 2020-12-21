#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from goal_marker import goalMarkers
from std_msgs.msg import Header
import rospy
import math
import tf

class Controller:
	def __init__(self):
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 1)
		rospy.wait_for_message("/base_pose_ground_truth", Odometry)
		self.pose_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.get_pose)
		self.following = False
		#self.laser_sub = rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
		self.listener=tf.TransformListener()
		self.cmd_vel = Twist()
		self.epsilon = 0.05
		self.collision_epsilon = 0.03
		self.position = None
		self.rate = rospy.Rate(100)
		self.markers = goalMarkers("/debug_goal", 1)
		self.arrived = False
		self.prev_position = None
		self.stuck = False
		self.turn_speed = 2
		self.forward_speed = 5
		self.robot_width = 3

		self.look_ahead = 0.05
		self.pursuit_intersect = None

	def laser_callback(self, data):
		regions = {
			'right':  min(min(data.ranges[0:216]), 10),
			'fright': min(min(data.ranges[217:432]), 10),
			'front':  min(min(data.ranges[433:648]), 10),
			'fleft':  min(min(data.ranges[649:865]), 10),
			'left':   min(min(data.ranges[866:1081]), 10),
		}

		if regions['front'] < self.collision_epsilon or regions['fleft'] < self.collision_epsilon or regions['fright'] < self.collision_epsilon:
			print("Obstacle too close!")
			self.stuck = True
			if regions['fleft'] < self.collision_epsilon:
				self.dir = -1
			else:
				self.dir = 1
			self.get_unstuck()
		elif regions['left'] < self.collision_epsilon and not self.stuck:
			print("left")
			self.stuck = True
			self.dir = -1
			point = self.get_laser_point(data)
			self.follow_obstacle(data, point)
		elif regions['right'] < self.collision_epsilon and not self.stuck:
			print("right")
			self.stuck = True
			self.dir = 1
			point = self.get_laser_point(data)
			self.follow_obstacle(data, point)
	
	def get_laser_point(self, data):
		a = data.angle_min
		i = data.angle_increment

		# initialise angles for scanning to lef to right of the robot
		# depending on wallfollowing direction
		if (self.dir == 1):
			start = -999
			end = 0
		else:
			start = 0
			end = 999

		# set ranges to arbitrary numbers initially
		self.rmin = 9999
		amin = 999
		for r in data.ranges:  # find closest point
			if(a <= end and a >= start and r < self.rmin):
				amin = a
				self.rmin = r
			a += i

		# compute point in laser frame of reference
		# and transform to robot frame
		x = self.rmin*math.cos(amin)
		y = self.rmin*math.sin(amin)

		ps = PointStamped(header=Header(stamp=rospy.Time(0), frame_id="/base_laser_link"), point=Point(x, y, 0))
		p = self.listener.transformPoint("/base_link", ps)
		return p		

	def get_unstuck(self):
		# TODO
		# Backwards
		for i in range(3):
			print("backwards")
			self.cmd_vel.linear.x = -0.3
			self.cmd_vel.angular.z = 0.3 * self.dir
			self.pub.publish(self.cmd_vel)
			self.rate.sleep()

		# Forwards slowly
		self.cmd_vel.linear.x = 0.1
		self.cmd_vel.angular.z = 0.3 * -self.dir
		self.pub.publish(self.cmd_vel)
		self.rate.sleep()
		self.stuck = False

	def follow_obstacle(self, data, point):
		# TODO
		self.following = True

		x = point.point.x
		y = point.point.y
		mag = self.rmin  # math.sqrt(x*x+y*y)

		# useful for later, magnitude of vector
		# tangent vector
		if(self.dir == 1):
			tx = -y
			ty = x
		else:
			tx = y
			ty = -x
		tx /= mag
		ty /= mag

		# where we should be going
		dx = x + tx - self.robot_width*x / mag
		dy = y + ty - self.robot_width*y / mag

		if (self.rmin < data.range_max):
			theta = math.atan2(dy, dx)
		elif (self.dir == 1):
			theta = -1
		else:
			theta = 1

		if (dx > 0.05):
			self.cmd_vel.linear.x = 0.5
		self.cmd_vel.angular.z = theta
		self.pub.publish(self.cmd_vel)

		self.stuck = False

	def get_pose(self, data):
		self.position = data.pose.pose.position
		quat = data.pose.pose.orientation
		_, _, self.theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

	def drive(self):
		if not self.arrived and not self.stuck:
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

				#rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=None)
				dx = abs(self.position.x - self.goal.x)
				dy = abs(self.position.y - self.goal.y)

				self.rate.sleep()

	def set_next_goal_pursuit(self):
		try:
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
		except:
			self.arrived = True


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
				self.arrived = True

			a = -math.atan2(py - self.position.y, px - self.position.x)
			c = (math.atan2(py - self.position.y, px - self.position.x) * self.position.x) - self.position.y
			x = abs(a * self.look_ahead + self.look_ahead + c) / (a**2 + 1)**0.5
			
			r = (self.look_ahead ** 2) / (2 * (x))
			self.cmd_vel.linear.x = self.sigmoid(abs(py)) * 2
			self.cmd_vel.angular.z = self.cmd_vel.linear.x / r


			print("p:", px, py)
			print(self.position.x, self.position.y)
			print(self.cmd_vel.linear.x, self.cmd_vel.angular.z)
			self.pub.publish(self.cmd_vel)

			#self.look_ahead = self.cmd_vel.linear.x * 0.2
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

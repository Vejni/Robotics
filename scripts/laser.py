#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rospy


def laser_callback(data):
	regions = {
		'right':  min(min(data.ranges[0:216]), 10),
		'fright': min(min(data.ranges[217:432]), 10),
		'front':  min(min(data.ranges[433:648]), 10),
		'fleft':  min(min(data.ranges[649:865]), 10),
		'left':   min(min(data.ranges[866:1081]), 10),
	}
	if regions['front'] < 0.05:
		print("obstacle detected")
		get_unstuck()

def get_unstuck():
	global cmd_vel
	cmd_vel.linear.x = -0.08
	cmd_vel.angular.z = 0.2
	pub.publish(cmd_vel)
	rate.sleep()

cmd_vel = Twist()
rospy.init_node("laser")
rate = rospy.Rate(10)
rospy.Subscriber("/base_scan", LaserScan, laser_callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.spin()

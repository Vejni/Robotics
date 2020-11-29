#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry


def callback(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z
	rospy.loginfo('x:{}, y:{}, z:{}'.format(x, y, z))


def main():
	rospy.init_node('location_monitor')
	rospy.Subscriber("/base_pose_ground_truth", Odometry, callback)
	rospy.spin()


if __name__ == "__main__":
	main()

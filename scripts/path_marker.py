#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def draw_believed_path(data):
	global believed_path
	believed_path.header = data.header
	pose = PoseStamped()
	pose.header = data.header
	pose.pose = data.pose.pose
	believed_path.poses.append(pose)
	believed_pub.publish(believed_path)

def draw_true_path(data):
	global true_path
	true_path.header = data.header
	pose = PoseStamped()
	pose.header = data.header
	pose.pose = data.pose.pose
	true_path.poses.append(pose)
	truth_pub.publish(true_path)


if __name__ == "__main__":
	believed_path = Path()
	true_path = Path()
	rospy.init_node("path_node")
	rate = rospy.Rate(1)
	rate.sleep()
	while not rospy.is_shutdown():
		# True Path
		try:
			print("hi")
			truth_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, draw_true_path)
			truth_pub = rospy.Publisher("/path_truth", Path, queue_size=10)
		except Exception as e:
			print("Believed position exception: ", e)

		# Believed Path
		try:
			believed_sub = rospy.Subscriber("/odom", Odometry, draw_believed_path)
			believed_pub = rospy.Publisher("/path_believed", Path, queue_size=10)
		except Exception as e:
			print("Believed position exception: ", e)

		rate.sleep()
		rospy.spin()


#!/usr/bin/env python

import rospy


if __name__ == "__main__":
	rospy.init_node("path_planning")
	while not rospy.is_shutdown():
		goal = rospy.get_param("/goal10")
		print(goal)
		rospy.spin()

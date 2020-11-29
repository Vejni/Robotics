#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


rospy.init_node('driver')
stop = Twist()
go = Twist()
go.linear.x = 0.5  # 0.5m/s forward speed
turn = Twist()
turn.angular.z = 1 # 1 rad/s twist speed
start_turn_time = rospy.Time.now()
end_turn_time = rospy.Time.now()
rate = rospy.Rate(10)


def obstacle_callback(data):
    global start_turn_time
    global end_turn_time
    if rospy.Time.now() > end_turn_time:  # we're not turning
        start_turn_time = rospy.Time.now()
        end_turn_time = rospy.Time.now()+rospy.Duration(random.random())

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    obstacle = rospy.Subscriber('nearby_obstacle', Bool, obstacle_callback, queue_size=1)
    start_drive_time = rospy.Time.now()
    driving = True
    while not rospy.is_shutdown():
        if rospy.Time.now() < end_turn_time:
            cmd_vel_pub.publish(turn)
        elif driving:
            cmd_vel_pub.publish(go)
        else:
            cmd_vel_pub.publish(stop)
        if rospy.Time.now() > start_drive_time + rospy.Duration(rospy.get_param("drive_time", 3)):
            start_drive_time = rospy.Time.now()
            driving = not driving
        rate.sleep()

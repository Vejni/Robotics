#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan


def scanner_callback(laserscan):
    angle = math.atan(0.15/rospy.get_param("min_range", 0.5))
    start_index = int((-angle-laserscan.angle_min) / laserscan.angle_increment)
    end_index = int((angle-laserscan.angle_min)/laserscan.angle_increment)+1
    for i in laserscan.ranges[start_index:end_index]:
        if i < rospy.get_param("min_range", 0.5):
            pub.publish(True)


rospy.init_node('detector')
pub = rospy.Publisher('nearby_obstacle', Bool, queue_size=1)
sub = rospy.Subscriber("base_scan", LaserScan, scanner_callback, queue_size=1)


if __name__ == '__main__':
    rospy.spin()

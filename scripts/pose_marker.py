#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MarkerBasics(object):

    def __init__(self, truth):
        if truth:
            self.marker_objectlisher = rospy.Publisher(
                '/marker_basic_truth',
                Marker,
                queue_size=1
            )
            self.truth = True

        else:
            self.marker_objectlisher = rospy.Publisher(
                '/marker_basic_believed',
                Marker,
                queue_size=1
            )
            self.truth = False

    def init_marker(self, position, orientation, index=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "map"
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = "haro"
        self.marker_object.id = index
        self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD

        self.marker_object.pose.orientation = orientation
        self.marker_object.pose.position = position
        self.marker_object.scale.x = 0.5
        self.marker_object.scale.y = 0.05
        self.marker_object.scale.z = 0.05

        if self.truth:
            self.marker_object.color.r = 0.0
            self.marker_object.color.g = 0.0
            self.marker_object.color.b = 1.0
        else:
            self.marker_object.color.r = 0.0
            self.marker_object.color.g = 1.0
            self.marker_object.color.b = 0.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)

    def start(self):

        self.marker_objectlisher.publish(self.marker_object)
     


def truthpos(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    marker = MarkerBasics(truth=True)
    marker.init_marker(position, orientation)
    marker.start()


def believedpos(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    marker = MarkerBasics(truth=False)
    marker.init_marker(position, orientation, index=1)
    marker.start()


if __name__ == '__main__':
    rospy.init_node('marker_basic', anonymous=True)
    rate = rospy.Rate(1)
    rate.sleep()

    try:
        rospy.Subscriber("/base_pose_ground_truth", Odometry, truthpos)
    except Exception as e:
        print("Truth position exception: ", e)

    try:
        rospy.Subscriber("/odom", Odometry, believedpos)
    except Exception as e:
        print("Believed position exception: ", e)

    rospy.spin()


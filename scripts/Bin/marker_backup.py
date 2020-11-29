#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MarkerBasics(object):

    def __init__(self, truth):
        if truth:
                self.marker_objectlisher = rospy.Publisher('/marker_basic_truth', Marker, queue_size=1)
                self.rate = rospy.Rate(1)
                self.init_marker(frame="base_footprint", index=0, z_val=0)
        else:
                self.marker_objectlisher = rospy.Publisher('/marker_basic_believed', Marker, queue_size=1)
                self.rate = rospy.Rate(1)
                self.init_marker(frame="odom", index=1, z_val=0)

    
    def init_marker(self, frame, index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = frame
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = "haro"
        self.marker_object.id = index
        self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD
        
        my_point = Point()
        my_point.z = z_val
        self.marker_object.pose.position = my_point
        
        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 0.5
        self.marker_object.scale.y = 0.05
        self.marker_object.scale.z = 0.05
    
        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0
            
        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)
    
    def start(self):
        while not rospy.is_shutdown():
            self.marker_objectlisher.publish(self.marker_object)
            self.rate.sleep()
   

if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_truth = MarkerBasics(truth=True)
    markerbasics_believed = MarkerBasics(truth=False)
    try:
        markerbasics_truth.start()
        markerbasics_believed.start()
    except rospy.ROSInterruptException:
        pass




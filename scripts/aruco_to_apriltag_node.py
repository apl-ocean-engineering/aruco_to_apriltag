#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from apriltag_msgs.msg import ApriltagArrayStamped, Apriltag
from geometry_msgs.msg import Point

# Global variable to store corner points from fiducial_vertices
corner_data = {}

# Callback for fiducial_vertices
def fiducial_vertices_callback(vertices):
    global corner_data
    corner_data = {}
    
    for fiducial in vertices.fiducials:
        fiducial_id = fiducial.fiducial_id

        corners = [Point(), Point(), Point(), Point()]
        corners[0].x, corners[0].y = fiducial.x0, fiducial.y0
        corners[1].x, corners[1].y = fiducial.x1, fiducial.y1
        corners[2].x, corners[2].y = fiducial.x2, fiducial.y2
        corners[3].x, corners[3].y = fiducial.x3, fiducial.y3
        
        corner_data[fiducial_id] = corners

# Callback for fiducial_transforms
def fiducial_to_apriltag_callback(data):
    apriltag_array_msg = ApriltagArrayStamped()
    apriltag_array_msg.header.stamp = rospy.Time.now()  
    apriltag_array_msg.header.frame_id = data.header.frame_id 
    
    for transform in data.transforms:
        apriltag_msg = Apriltag()
        apriltag_msg.id = transform.fiducial_id
        apriltag_msg.family = "aruco"
        apriltag_msg.hamming = 0
        apriltag_msg.border = 1
        apriltag_msg.bits = 6

        apriltag_msg.center = Point()
        apriltag_msg.center.x = transform.transform.translation.x
        apriltag_msg.center.y = transform.transform.translation.y

        if transform.fiducial_id in corner_data:
            apriltag_msg.corners = corner_data[transform.fiducial_id]
        else:
            rospy.logwarn(f"No corner data available for fiducial ID: {transform.fiducial_id}")
            apriltag_msg.corners = [Point() for _ in range(4)] 

        apriltag_array_msg.apriltags.append(apriltag_msg)

    apriltag_pub.publish(apriltag_array_msg)

if __name__ == '__main__':
    rospy.init_node('fiducial_to_apriltag_converter')

    fiducial_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_to_apriltag_callback)
    
    fiducial_vertices_sub = rospy.Subscriber('/fiducial_vertices', FiducialArray, fiducial_vertices_callback)

    apriltag_pub = rospy.Publisher('/apriltag_detections', ApriltagArrayStamped, queue_size=10)

    rospy.spin()

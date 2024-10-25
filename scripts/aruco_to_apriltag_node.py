#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialArray
from apriltag_msgs.msg import ApriltagArrayStamped, Apriltag
from geometry_msgs.msg import Point

def fiducial_vertices_callback(vertices):
    apriltag_array_msg = ApriltagArrayStamped()
    apriltag_array_msg.header.stamp = vertices.header.stamp  
    apriltag_array_msg.header.frame_id = vertices.header.frame_id  

    for fiducial in vertices.fiducials:
        apriltag_msg = Apriltag()
        apriltag_msg.id = fiducial.fiducial_id
        apriltag_msg.family = "4X4_250" 

        # Store the four corner points
        corners = [Point(), Point(), Point(), Point()]
        corners[0].x, corners[0].y = fiducial.x0, fiducial.y0
        corners[1].x, corners[1].y = fiducial.x1, fiducial.y1
        corners[2].x, corners[2].y = fiducial.x2, fiducial.y2
        corners[3].x, corners[3].y = fiducial.x3, fiducial.y3

        apriltag_msg.corners = corners

        # Calculate the center by averaging the corners
        apriltag_msg.center = Point()
        apriltag_msg.center.x = sum(corner.x for corner in corners) / 4.0
        apriltag_msg.center.y = sum(corner.y for corner in corners) / 4.0

        apriltag_array_msg.apriltags.append(apriltag_msg)

    # Publish the apriltag array
    apriltag_pub.publish(apriltag_array_msg)

if __name__ == '__main__':
    rospy.init_node('fiducial_to_apriltag_converter')

    # Subscribe to the fiducial vertices
    fiducial_vertices_sub = rospy.Subscriber('/fiducial_vertices', FiducialArray, fiducial_vertices_callback)

    # Publisher for the ApriltagArrayStamped detections
    apriltag_pub = rospy.Publisher('/apriltag_detections', ApriltagArrayStamped, queue_size=10)

    rospy.spin()

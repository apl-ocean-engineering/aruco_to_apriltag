#!/usr/bin/env python3

from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage
from pathlib import Path
import cv2
from apriltag_msgs.msg import ApriltagArrayStamped, Apriltag
from geometry_msgs.msg import Point
import rospy
import rosbag
import argparse

def extract_and_generate_apriltag_msgs(in_bag_path, out_bag_path, image_topic, encoding="bgr8"):

    # Set up the ArUco dictionary and parameters for detection
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    out_bag = rosbag.Bag(out_bag_path, 'w')  # Open output bag for writing

    # Open the bag file with AnyReader
    with AnyReader([Path(in_bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == image_topic]
        
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # Deserialize the message
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # Convert ROS message to OpenCV image
            cv_image = message_to_cvimage(msg, encoding)
            
            # Detect ArUco markers
            corners, ids, rejected = detector.detectMarkers(cv_image)
            
            if ids is not None:
                # Create the ApriltagArrayStamped message
                apriltag_array_msg = ApriltagArrayStamped()
                apriltag_array_msg.header.stamp = msg.header.stamp  
                apriltag_array_msg.header.frame_id = "camera_frame"  # TODO: Update this

                # Loop over detected ArUco markers to populate ApriltagArrayStamped
                for i, corner in enumerate(corners):
                    apriltag_msg = Apriltag()
                    apriltag_msg.id = int(ids[i])
                    apriltag_msg.family = "4X4_250"
                    
                    # Calculate the center point as the average of the corners
                    center_x = sum([c[0] for c in corner[0]]) / 4.0
                    center_y = sum([c[1] for c in corner[0]]) / 4.0
                    apriltag_msg.center = Point(center_x, center_y, 0)

                    # Set the corners in the apriltag_msg
                    apriltag_msg.corners = [
                        Point(c[0], c[1], 0) for c in corner[0]
                    ]

                    # Add to the array message
                    apriltag_array_msg.apriltags.append(apriltag_msg)

                # Write to the output bag
                ros_time = rospy.Time(msg.header.stamp.sec, msg.header.stamp.nanosec)
                apriltag_array_msg.header.stamp = ros_time
                out_bag.write('/apriltag_detections', apriltag_array_msg, t=ros_time)

    out_bag.close()
    print(f"Output bag written to: {out_bag_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process a bagfile of images to create a bagfile of AprilTag detections.')
    parser.add_argument('input', help='Path to the input bagfile containing images.')
    parser.add_argument('--output', required=True, help='Path to the output bagfile for AprilTag detections.')
    parser.add_argument('--image_topic', default='/trisect/stereo/left/image_rect', help='Image topic to process')
    args = parser.parse_args()

    extract_and_generate_apriltag_msgs(args.input, args.output, args.image_topic)

#!/usr/bin/env python

import os
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

cv_bridge = CvBridge()

def extract_images(bag_file, output_dir, image_topic):
    # Initialize the ROS node
    rospy.init_node('extract_depth_images', anonymous=True)

    # Open the rosbag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            if msg._type == 'sensor_msgs/CompressedImage':
                depth_header_size = 12
                raw_data = msg.data[depth_header_size:]
                depth_img_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)

                timestamp = str(t.to_nsec())
                output_file = os.path.join(output_dir, timestamp + '.png')
                cv2.imwrite(output_file, depth_img_raw)
                print(f"Saved {output_file}")

if __name__ == "__main__":
    # Parameters
    bag_file = "demo_area_nuvo.bag"
    output_dir = "darko_demo/depths/"
    depth_image_topic = "/robot/k4a_top/depth/image_raw/compressedDepth"  # Change this to your image topic

    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Extract images
    extract_images(bag_file, output_dir, depth_image_topic)

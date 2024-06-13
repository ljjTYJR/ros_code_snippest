#!/usr/bin/env python
# deump ros image topic to images

import os
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def extract_images(bag_file, output_dir, image_topic):
    # Initialize the CvBridge
    bridge = CvBridge()

    # Open the rosbag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            if msg._type == 'sensor_msgs/CompressedImage':
                try:
                    # Convert the ROS CompressedImage message to an OpenCV image
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    # Create the filename based on the timestamp
                    timestamp = str(t.to_nsec())
                    filename = os.path.join(output_dir, timestamp + ".jpg")

                    # Save the image
                    cv2.imwrite(filename, cv_image)

                    print(f"Saved image: {filename}")
                except CvBridgeError as e:
                    print(f"Failed to convert image: {e}")

if __name__ == "__main__":
    # Parameters
    bag_file = "demo_area_nuvo.bag"
    output_dir = "darko_images/"
    image_topic = "/robot/k4a_top/rgb/image_raw/compressed"  # Change this to your image topic

    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Extract images
    extract_images(bag_file, output_dir, image_topic)

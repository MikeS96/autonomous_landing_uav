#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()


def depth_callback(msg_depth):
    print("Received a depth image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_depth = bridge.imgmsg_to_cv2(msg_depth, "32FC1")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg ros rosbag play -r 0.2 take_4.bag
        cv_depth_array = np.array(cv2_depth, dtype = np.dtype('f8'))
        time = msg_depth.header.stamp
        print('saving depth image at ' + 'secs_'+str(time.secs)+'nsecs_'+str(time.nsecs))
        np.savetxt('depth/secs_'+str(time.secs)+'nsecs_'+str(time.nsecs)+'.dat', cv_depth_array)
        #rospy.sleep(1)

def main():
    rospy.init_node('depth_listener')
    # Define your image topic
    depth_topic = "/camera/depth/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(depth_topic, Image, depth_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
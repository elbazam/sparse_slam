#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def normalize_depth_image(depth_image, max_range):
    depth_image = (depth_image/max_range)*255
    return np.round(depth_image).astype(np.uint8)

def _cv_bridge_callback(img, bridge, pub_compresse):
    # encoding of simulated depth image: 32FC1, encoding of real depth image: 16UC1
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="32FC1")
    cv_image = np.array(cv_image, dtype=np.float)    
    depth_image = normalize_depth_image(cv_image, 4.0)
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    compressed_img = np.array(cv2.imencode('.jpg', depth_image)[1])
    msg.data = compressed_img.tostring()
    pub_compresse.publish(msg)

rospy.init_node('cv_bridg', anonymous=True)
bridge = CvBridge()
pub_compresse = rospy.Publisher("compressed", CompressedImage, queue_size=2)
rospy.Subscriber("image_raw", Image, lambda img: _cv_bridge_callback(img, bridge, pub_compresse), queue_size=2)
rospy.loginfo("cv_bridg node")

rospy.spin()

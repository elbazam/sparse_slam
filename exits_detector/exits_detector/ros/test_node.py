#!/usr/bin/env python3

import numpy as np
import cv2
import time
import rospy
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegmentList
from exits_detector.api import *





def line_segments_callback(lines_msg): 
    line_intersections = find_optional_exits(lines_msg, 0.1)
    
    scan = rospy.wait_for_message('/scan',LaserScan)
    line_intersections = filter_exits(line_intersections,scan)
    print(line_intersections)
    
    M = visualize(scan,line_intersections)
    cv2.imshow('M',cv2.resize(M,(512,512)))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
    
   

def shutdown():
    cv2.destroyAllWindows()   

rospy.init_node('ScanFlow', anonymous=True)
#rospy.Subscriber('/scan',LaserScan, scan_callback, queue_size = 1)
rospy.Subscriber('/line_segments',LineSegmentList, line_segments_callback, queue_size = 1)
rospy.on_shutdown(shutdown)
rospy.spin()
rospy.signal_shutdown("ScanFlow shutdown...")
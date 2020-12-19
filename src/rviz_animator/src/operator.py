#!/usr/bin/env python

import rospy
import sys
import rospy
import tf2_ros
import traceback
import numpy as np

from tf2_geometry_msgs import do_transform_pose
from rviz_animator.msg import Keyframe
from geometry_msgs.msg import (    
	Point,
	Quaternion,
	Pose
)

def talker():
  pub = rospy.Publisher('keyframe_sequence', Keyframe, queue_size=10)
  
  r = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
    pub_input = raw_input("Please enter a line of text and press <Enter>: ")
    
    # Publish our string to the 'keyframe_sequence' topic
    pub.publish(pub_input, rospy.get_time())
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
if __name__ == '__main__':
  rospy.init_node('operator', anonymous=True)
  try:
    talker()
  except rospy.ROSInterruptException: pass

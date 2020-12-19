#!/usr/bin/env python

import rospy

from sensor_msgs.msg import (
    JointState
)
from std_msgs.msg import (
    Float32MultiArray
)

def callback(message):
    labels = message.name
    angles = message.position

    if len(angles) == 10: # Joint State slider has 10 angles
        real_joint_indices = [0, 1, 4, 7, 8, 9]
        real_angles = [angles[i] for i in real_joint_indices]
        real_labels = [labels[i] for i in real_joint_indices]

        angles, labels = real_angles, real_labels
    elif len(angles) == 6: # Choreographer has 6 angles
        pass
    pub_robot.publish(Float32MultiArray(data=angles))
    print("Running callback with:\n labels: {}\nangles: {}".format(labels, angles))

if __name__ == "__main__":
    rospy.init_node('robot_controller', anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    pub_robot = rospy.Publisher('arduino_joints', Float32MultiArray, queue_size=10)

    print("Ready to send joints to Arduino!")

    rospy.spin()
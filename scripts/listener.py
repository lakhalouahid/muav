#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def callback(image: Image):
    rospy.loginfo(rospy.get_caller_id() + 'I heard (%d, %d)', image.height, image.width)

def listener():
    rospy.init_node('perception', anonymous=True)
    rospy.Subscriber('front_cam/camera/image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

from cv_bridge import CvBridge

from uav_base.UAVBase import UAVBase
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class Camera():
  def __init__(self, fx=800, fy=800, h=20000):
    self.h = h
    self.K = np.array([[1.0, 0.0, 240.5], [0.0, 1.0,  320.5], [0, 0, 1]])
    self.Kinv = np.linalg.inv(self.K)


class BlobDetector(object):
  def __init__(self, fig, axe):
    super(BlobDetector, self)
    self.fig, self.axe = fig, axe

  def detect(self, image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    target = rospy.get_param("target")
    if target == "blue":
        mask = cv2.inRange(hsv_image, (90, 50, 70), (128, 255,255))
    else:
        mask1 = cv2.inRange(hsv_image, (0, 50, 70), (10, 255,255))
        mask2 = cv2.inRange(hsv_image, (160, 50, 70), (180, 255,255))
        mask = mask1 + mask2
    np_image = np.array(image)
    center = tuple([np.mean(axis, dtype=np.int64) for axis in np.where(mask == 255)])
    print(center)
    return center




class FixedUAV(UAVBase):
    def __init__(self, topic, fig, axe):
        super(FixedUAV, self).__init__(topic)
        self.detector = BlobDetector(fig, axe)
        self.bridge = CvBridge()

    def process_image(self, image: Image):
        self.cam = Camera(fx=640, fy=480, h=self.position.z)
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        np_image = np.asarray(cv_image, dtype=np.uint8)
        target_center = self.detector.detect(cv_image)
        self.target_position = self.cam.Kinv @ np.array([[target_center[0]], [target_center[1]], [1]])
        print(self.target_position)
        self.target_position[1] = -self.target_position[1] * self.cam.h / 268.51188197672957
        self.target_position[0] = -self.target_position[0] * self.cam.h / 268.51188197672957
        self.target_position[2] = self.cam.h
        print(self.target_position)
        
        



def main():
    fig = plt.figure()
    axe = fig.add_subplot(111)
    uav = FixedUAV('fixed/cmd_vel', fig, axe)
    rospy.Subscriber('fixed/ground_truth/state', Odometry, uav.update_state)
    rospy.Subscriber('fixed/downward_cam/camera/image', Image, uav.process_image)
    rospy.init_node('fixed', anonymous=True)
    rate = rospy.Rate(1)
    fixed_position = Point(0, 0, 4)
    uav.move(fixed_position)
    while not rospy.is_shutdown():
        rate.sleep() 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

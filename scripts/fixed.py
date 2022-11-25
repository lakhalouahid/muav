#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge

from uav_base.UAVBase import UAVBase
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class Camera():
  def __init__(self, fx=800, fy=800, h=20000):
    self.h = h
    self.K = np.array([[fx, 0, fx // 2], [0, fy, fy // 2], [0, 0, 1]])
    self.Kinv = np.linalg.inv(self.K)
    self.R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1], [0, 0, 0]])
    self.Rinv = np.concatenate([self.R[:3].T, np.zeros((1, 3))], axis=0)
    self.t = np.array([[0], [0], [self.h], [1]])
    self.tinv = np.concatenate([-self.Rinv[:3] @ self.t[:3], np.array([[1]])], axis=0)
    self.C = np.concatenate([self.R, self.t], axis=1)
    self.Cinv = np.concatenate([self.Rinv, self.tinv], axis=1)


class BlobDetector(object):
  def __init__(self):
    super(BlobDetector, self)

  def detect(self, image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    target = rospy.get_param("")
    if target == "blue":
        mask = cv2.inRange(hsv_image, (36, 0, 0), (70, 255,255))
    else:
        mask = cv2.inRange(hsv_image, (36, 0, 0), (70, 255,255))
    center = tuple([np.mean(axis, dtype=np.int64) for axis in np.where(mask == 255)])
    return center




class FixedUAV(UAVBase):
    def __init__(self, *args, **kwargs):
        super(FixedUAV, self).__init__(*args, **kwargs)
        self.detector = BlobDetector()
        self.bridge = CvBridge()

    def process_image(self, image: Image):
        self.cam = Camera(1000, 1000, self.position.z)
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # np_image = np.asarray(cv_image, dtype=np.uint8)
        target_center = self.detector.detect(cv_image)
        self.target_position = self.cam.Kinv @ np.array([[self.cam.h * target_center[0]], [self.cam.h * target_center[1]], [self.cam.h]])
        
        



def main():
    uav = FixedUAV('fixed/cmd_vel')
    rospy.Subscriber('fixed/ground_truth/state', Odometry, uav.update_state)
    rospy.Subscriber('fixed/front_cam/camera/image', Image, uav.process_image)
    rospy.init_node('fixed', anonymous=True)
    rate = rospy.Rate(1)
    fixed_position = Point(0, 0, 5)
    uav.move(fixed_position)
    while not rospy.is_shutdown():
        rate.sleep() 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

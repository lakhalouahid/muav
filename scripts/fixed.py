#!/usr/bin/env python3

import rospy
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt

from cv_bridge import CvBridge

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from sensor_msgs.msg import Image
from muav.msg import Tracking


class Camera():
  def __init__(self, h=20000):
    self.h = h
    self.K = np.array([[268.51188197672957, 0.0, 240.5], [0.0, 268.51188197672957, 320.5], [0, 0, 1]])
    self.Kinv = np.linalg.inv(self.K)


class BlobDetector(object):
  def __init__(self):
      pass

  def detect(self, image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #target = rospy.get_param("target")
    target = rospy.get_param("current_color")
    if target == "blue":
        mask = cv2.inRange(hsv_image, (90, 50, 70), (128, 255,255))
    else:
        mask = cv2.inRange(hsv_image, (0, 50, 70), (10, 255,255)) + \
                cv2.inRange(hsv_image, (160, 50, 70), (180, 255,255))
    center = np.array([np.mean(axis, dtype=np.int64) for axis in np.where(mask == 255)])
    return np.array([center[0], center[1], 1])




class FixedUAV():
    def __init__(self, fixed_position):
        self.fixed_position = fixed_position
        self.detector = BlobDetector()
        self.bridge = CvBridge()
        self.tracking_pose_cmd = rospy.Publisher('tracking/command/pose', PoseStamped, queue_size=1)
        #publisher for custom msg
        self.color_pub = rospy.Publisher('current_color', Tracking, queue_size=1)


    def process_image(self, image: Image):
        self.cam = Camera(h=self.fixed_position.z)
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        target_center = self.detector.detect(cv_image)
        target_pose = -self.cam.Kinv @ target_center * self.cam.h
        if abs(target_pose[0]) < 10 and abs(target_pose[1]) < 10:
            print(target_pose[0], target_pose[1])
            h = Header(); h.stamp = rospy.Time.now()
            pose = PoseStamped(header=h, pose=Pose(Point(target_pose[0].item(), target_pose[1].item(), 2), Quaternion(0, 0, 0, 1)))
            self.tracking_pose_cmd.publish(pose)
            msg = Tracking()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = rospy.get_param("current_color")
            msg.X = target_pose[0]
            msg.Y = target_pose[1]
            self.color_pub.publish(msg)


def main():
    rospy.init_node('fixed', anonymous=True)
    fixed_position = Point(0, 0, 8)
    fixed_uav = FixedUAV(fixed_position)
    rospy.Subscriber('fixed/downward_cam/camera/image', Image, fixed_uav.process_image, queue_size=1)
    pose_cmd = rospy.Publisher('fixed/command/pose', PoseStamped, queue_size=100)
    while not rospy.is_shutdown():
        h = Header(); h.stamp = rospy.Time.now()
        fixed_pose = PoseStamped(header=h, pose=Pose(fixed_position, Quaternion(0, 0, 0, 1)))
        pose_cmd.publish(fixed_pose)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

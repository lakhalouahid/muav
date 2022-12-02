import cv2
import numpy as np
from matplotlib import pyplot as plt

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

  def detect(self, img):
    threshold = 100
    _, mask1 = cv2.threshold(img[:, :, 0], threshold, 255, cv2.THRESH_BINARY)
    _, mask2 = cv2.threshold(img[:, :, 1], threshold, 255, cv2.THRESH_BINARY)
    center1 = tuple([np.mean(axis, dtype=np.int64) for axis in np.where(mask1 == 255)])
    center2 = tuple([np.mean(axis, dtype=np.int64) for axis in np.where(mask2 == 255)])
    results = {}
    results["mask1"] = mask1
    results["center1"] = center1
    results["mask2"] = mask2
    results["center2"] = center2
    return results

if __name__ == "__main__":
  blob_detector = BlobDetector()
  fixed_cam = Camera(1000, 1000)
  img = np.zeros(shape=(600, 600, 3), dtype=np.uint8)
  img = cv2.circle(img, (150, 150), 100, (255, 0, 0), -1)
  img = cv2.circle(img, (450, 450), 100, (0, 255, 0), -1)
  results = blob_detector.detect(img)
  mask1 = results["mask1"]
  center1 = results["center1"] = (0, 0)
  mask2 = results["mask2"]
  center2 = results["center2"]
  pos1 = fixed_cam.Kinv @ np.array([[fixed_cam.h * center1[0]], [fixed_cam.h * center1[1]], [fixed_cam.h]])
  print(pos1)

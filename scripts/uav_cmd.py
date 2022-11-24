#!/usr/bin/env python3

import rospy

from utils import dist_position, dist_orientation, sym_clip, UAVBase
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry






def main():
    uav = UAVBase('uav2/cmd_vel')
    rospy.Subscriber('uav2/ground_truth/state', Odometry, uav.update_state)
    rospy.init_node('uav2_cmd', anonymous=True)
    while True:
      target_position = Point(4.0, 4.0, 2.0)
      uav.move(target_position)

      target_position = Point(0.0, 0.0, 4.0)
      uav.move(target_position)
    rospy.spin()
    # while not rospy.is_shutdown():
    #   rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

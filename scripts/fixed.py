#!/usr/bin/env python3

import rospy

from uav_base.UAVBase import UAVBase
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry






def main():
    uav = UAVBase('fixed/cmd_vel')
    rospy.Subscriber('fixed/ground_truth/state', Odometry, uav.update_state)
    rospy.init_node('fixed', anonymous=True)
    fixed_position = Point(0, 0, 5)
    uav.move(fixed_position)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

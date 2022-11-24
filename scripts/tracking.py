#!/usr/bin/env python3

import rospy

from uav_base.UAVBase import UAVBase
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry



def main():
    uav = UAVBase('tracking/cmd_vel')
    rospy.Subscriber('tracking/ground_truth/state', Odometry, uav.update_state)
    rospy.Subscriber('tracking/pose_cmd', Pose, uav.pose_cmd)
    rospy.init_node('tracking', anonymous=True)
    position = Point(0, 0, 2)
    uav.move(position)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

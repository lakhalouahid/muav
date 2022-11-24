#!/usr/bin/env python3

import rospy

from utils import dist_position, dist_orientation, sym_clip, UAVBase
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry






def main():
    uav = UAVBase('uav1/cmd_vel')
    rospy.Subscriber('uav1/ground_truth/state', Odometry, uav.update_state)
    uav2_pose_cmd = rospy.Publisher('/uav2/pose_cmd', Pose)
    rospy.init_node('uav1_cmd', anonymous=True)
    target_position = Point(0, 0, 4)
    uav.move(target_position)
    uav2_pose_cmd.publish(Pose(Point(8, 8, 2), Quaternion(0, 0, 0, 1)))
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

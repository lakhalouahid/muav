#!/usr/bin/env python3
import roslib; roslib.load_manifest('muav')
import rospy

from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from std_msgs.msg import Header



def main():
    rospy.init_node('tracking', anonymous=True)
    rate = rospy.Rate(1)
    pose_cmd = rospy.Publisher('tracking/command/pose', PoseStamped, queue_size=1)
    initial_position = Point(0, 0, 2)
    h = Header(); h.stamp = rospy.Time.now()
    initial_pose = PoseStamped(header=h, pose=Pose(initial_position, Quaternion(0, 0, 0, 1)))
    pose_cmd.publish(initial_pose)
    while not rospy.is_shutdown():
        rate.sleep() 



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

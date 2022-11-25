#!/usr/bin/env python3
import roslib; roslib.load_manifest('muav')
import rospy

from uav_base.UAVBase import UAVBase
from muav.srv import TrackingPoseCmd,TrackingPoseCmdResponse
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry


class TrackingUAV(UAVBase):
    def __init__(self, *args, **kwargs):
        super(TrackingUAV, self).__init__(*args, **kwargs,)

    def cmd_position(self, req):
        self.pose_cmd(Point(req.x, req.y, req.z))
        return TrackingPoseCmdResponse(0);


def main():
    uav = TrackingUAV('tracking/cmd_vel')
    rospy.Subscriber('tracking/ground_truth/state', Odometry, uav.update_state)
    rospy.init_node('tracking', anonymous=True)
    s = rospy.Service("tracking/pose_cmd", TrackingPoseCmd,  uav.cmd_position)
    position = Point(0, 0, 2)
    uav.move(position)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

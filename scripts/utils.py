import rospy
import math
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry


def dist_position(uav, act_position, delta=1e-2):
  dx = abs(uav.position.x - act_position.x)
  dy = abs(uav.position.y - act_position.y)
  dz = abs(uav.position.z - act_position.z)
  speed = math.sqrt(uav.linear_velocity.x * uav.linear_velocity.x + uav.linear_velocity.y * uav.linear_velocity.y + uav.linear_velocity.z * uav.linear_velocity.z)
  return max([dx, dy, dz]) <= delta and speed <= 0.5



def dist_orientation(uav, act_orientation, delta=1e-2):
  dx = abs(uav.orientation.x - act_orientation.x)
  dy = abs(uav.orientation.y - act_orientation.y)
  dz = abs(uav.orientation.z - act_orientation.z)
  dw = abs(uav.orientation.w - act_orientation.w)
  return max([dx, dy, dz, dw]) <= delta

def sym_clip(val, clip):
  return min(max(val, -clip), clip)

class UAVBase():
  def __init__(self, cmd_vel_topic, max_speed=Vector3(5, 5, 5)):
    self.position = Point()
    self.orientation = Quaternion()
    self.linear_velocity = Vector3()
    self.angular_velocity = Vector3()
    self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    self.max_speed = max_speed

  def update_state(self, state):
    self.position = state.pose.pose.position
    self.orientation = state.pose.pose.orientation
    self.linear_velocity = state.twist.twist.linear
    self.angular_velocity = state.twist.twist.angular

  def move(self, position, orientation=Quaternion(0, 0, 0, 1), interval=1):
    print(self.position)
    ispositioned, isoriented = False, False
    while not (ispositioned or isoriented):
      print(self.orientation)
      if dist_position(self, position, 3e-2):
        ispositioned = True

      dx, dy, dz = self.position.x - position.x, self.position.y - position.y, self.position.z - position.z
      dx, dy, dz = sym_clip(dx, self.max_speed.x), sym_clip(dy, self.max_speed.y), sym_clip(dz, self.max_speed.z)

      ref_euler_orientation = tf.transformations.euler_from_quaternion([orientation.x , orientation.y, orientation.z, orientation.w])
      act_euler_orientation = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
      # ewx, ewy, ewz = ref_euler_orientation[0] - act_euler_orientation[0], ref_euler_orientation[1] - act_euler_orientation[1] ref_euler_orientation[2] - act_euler_orientation[2]
      ewx, ewy, ewz = 0, 0, 0

      linear_velocity = Vector3(-dx / (1.5*interval), -dy / (1.5*interval), -dz / (1.5*interval))
      # angular_velocity = Vector3(-ewx*abs(ewx) / (interval**2), -ewy*abs(ewy) / (interval**2), -ewz * abs(ewz) / (interval**2))

      self.cmd_vel(Twist(linear_velocity, Vector3()), interval)

    self.cmd_vel(Twist(Vector3(), Vector3()), interval)
    
  def pose_cmd(self, pose):
    self.move(pose.position, pose.orientation)

  def cmd_vel(self, twist, interval):
    self.cmd_vel_pub.publish(twist)
    rospy.sleep(interval)

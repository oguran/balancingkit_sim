import rospy
import struct
import collections
from sensor_msgs.msg import Imu

Vector = collections.namedtuple('Vector', 'x y z')

class LSM6(object):

  def __init__(self):
    self.g = Vector(0, 0, 0)
    self.a = Vector(0, 0, 0)

  def imu_callback(self, data):
    self.g = Vector(data.angular_velocity)
    self.a = Vector(data.linear_acceleration)

  def enable(self):
    rospy.init_node("virtual_lsm6", anonymous=True)
    rospy.Subscriber('/imu/data', Imu, self.imu_callback)

  def read(self):
    # nothing to do.

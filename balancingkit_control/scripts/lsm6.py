#import smbus
import rospy
import struct
import collections
from sensor_msgs.msg import Imu

Vector = collections.namedtuple('Vector', 'x y z')

class LSM6(object):

  def __init__(self, slave_addr = 0b1101011):
    self.g = Vector(0, 0, 0)
    self.a = Vector(0, 0, 0)

  def enable(self):

  def read(self):

  def imu_callback():
    self.g = Vector(msg.angular_velocity)
    self.a = Vector(msg.linear_acceleration)

  def listener():
    rospy.init_node("virtual_lsm6", anonymous=True)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

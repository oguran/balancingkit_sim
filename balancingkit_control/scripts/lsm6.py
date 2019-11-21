#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
    self.g = Vector(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
    self.a = Vector(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
    print 'imu_callback self.g = ', self.g, '  imu_callback self.a = ', self.a

  def enable(self):
    #rospy.init_node("virtual_lsm6", anonymous=True)
    rospy.Subscriber('/imu/data', Imu, self.imu_callback)

  def read(self):
    # nothing to do.
    pass

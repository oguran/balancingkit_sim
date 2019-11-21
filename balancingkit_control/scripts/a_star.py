#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import struct
import time
import threading
import array
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

ASTER_ENC_CNT = 12
ASTAR_GEAR_RATIO = 111

class AStar:
  # This lock allows multiple threads in a program to use their own AStar
  # instances without interfering with each other's I2C communications.
  lock = threading.Lock()

  def __init__(self):
    self.position = np.array([0, 0], dtype=np.int16);
    self.raw_position = np.array([0.0, 0.0], dtype=np.float64);
    self.raw_last_position = np.array([0.0, 0.0], dtype=np.float64);
    self.raw_velocity_rpm = np.array([0.0, 0.0], dtype=np.float64);
    self.raw_torq = np.array([0.0, 0.0], dtype=np.float64)
    rospy.init_node("virtual_a_star", anonymous=True)
    rospy.Subscriber('/balancingkit_on_gazebo/joint_states', JointState, self.joint_state_callback)
    self.pub_l = rospy.Publisher('/balancingkit_on_gazebo/left_joint_effort_controller/command', Float64, queue_size=10)
    self.pub_r = rospy.Publisher('/balancingkit_on_gazebo/right_joint_effort_controller/command', Float64, queue_size=10)

  def joint_state_callback(self, data):
    self.raw_last_position = self.raw_position;
    self.raw_position = data.position;
    print 'joint_state_callback data = ', data
    # Following '50' is publish rate of joint_state_controller. @see contoroller.yaml
    self.raw_velocity_rpm[0] = (self.raw_last_position[0] - self.raw_position[0]) / (1.0/50.0) * 60.0;
    self.raw_velocity_rpm[1] = (self.raw_last_position[1] - self.raw_position[1]) / (1.0/50.0) * 60.0;

#  def read_unpack(self, address, size, format):
    # Ideally we could do this:
    #    byte_list = self.bus.read_i2c_block_data(SLAVE_ADDRESS, address, size)
    # But the AVR's TWI module can't handle a quick write->read transition,
    # since the STOP interrupt will occasionally happen after the START
    # condition, and the TWI module is disabled until the interrupt can
    # be processed.
    #
    # A delay of 0.0001 (100 us) after each write is enough to account
    # for the worst-case situation in our example code.
#    AStar.lock.acquire()
#    self.bus.write_byte(SLAVE_ADDRESS, address)
#    time.sleep(0.0001)
#    byte_list = [self.bus.read_byte(SLAVE_ADDRESS) for _ in range(size)]
#    AStar.lock.release()
#    return struct.unpack(format, bytes(byte_list))

#  def write_pack(self, address, format, *data):
#    data_array = list(struct.pack(format, *data))
#    AStar.lock.acquire()
#    self.bus.write_i2c_block_data(SLAVE_ADDRESS, address, data_array)
#    time.sleep(0.0001)
#    AStar.lock.release()

#  def leds(self, red, yellow, green):
#    self.write_pack(0, 'BBB', red, yellow, green)

#  def play_notes(self, notes):
#    self.write_pack(24, 'B15s', 1, notes.encode("ascii"))

  def motors(self, left, right):
    vm = np.array([left, right]);
    print '!!!!!!!!!!!!!!!!!!!!!! motor start vm = ', vm, ' !!!!!!!!!!!!!!!!!!!i!!!!!!!!'
#    self.write_pack(6, 'hh', left, right)
    self.raw_torq = 620/(6*85) * vm - 1/85 * self.raw_velocity_rpm
    self.raw_torq[0] = 1
    self.raw_torq[1] = 1
    print '!!!!!!!!!!!!!!!!!!!!!! motor publish torq = ', self.raw_torq, ' !!!!!!!!!!!!!!!!!!!i!!!!!!!!'
    self.pub_l.publish(self.raw_torq[0]);
    self.pub_r.publish(self.raw_torq[1]);

#  def read_buttons(self):
#    return self.read_unpack(3, 3, "???")

#  def read_battery_millivolts(self):
#    return self.read_unpack(10, 2, "H")

#  def read_analog(self):
#    return self.read_unpack(12, 12, "HHHHHH")

  def read_encoders(self):
    enc_raw_cnt = (self.raw_position * ASTER_ENC_CNT * ASTAR_GEAR_RATIO);
    self.position[0] = int(enc_raw_cnt[0]) & 0xFFFF;
    self.position[1] = int(enc_raw_cnt[1]) & 0xFFFF;
    return self.position

#  def test_read8(self):
#    self.read_unpack(0, 8, 'cccccccc')

#  def test_write8(self):
#    AStar.lock.acquire()
#    self.bus.write_i2c_block_data(SLAVE_ADDRESS, 0, [0,0,0,0,0,0,0,0])
#    time.sleep(0.0001)
#    AStar.lock.release()

#!/usr/bin/env python
import os
import copy
from collections import namedtuple

import rospy
import rospkg
rospack = rospkg.RosPack()

import actionlib
from xela_ros.msg import *
from xela_ros.srv import *
from xela_ros.xela_sensor import *

Forces = namedtuple('Forces', ('type', 'force', 'f_n', 'f_t', 'center_of_pressure'))
Forces.__new__.__defaults__ = (None,) * (len(Forces._fields) - 2) # Defaults for all but type and force


class SensorUnit(object):
  """This class encapsulates everything required for a single sensor unit

  Attributes:
    _board_id (int) = The id number of the sensor board this sensor belongs to
    _sensor_num (int) = The unique sensor number for this sensor
    base_pub (obj) = The ROS publisher for the base (calibration reading)
    data_pub (obj) = The ROS publisher for the raw data reading
    cntr_pub (obj) = The ROS publisher for the centered (calibrated) reading
    base (list) = The list containing the base (calibration) data
    data (list) = The list containing the raw data
    cntr (list) = The list containing the centered data

  """

  def __init__(self, board_id, sensor_num, sensor_base):
    self._board_id = board_id
    self._sensor_num = sensor_num
    self.base_pub = rospy.Publisher("~base_{}"    .format(self._sensor_num), XelaSensorStamped, queue_size=1)
    self.data_pub = rospy.Publisher("~data_{}"    .format(self._sensor_num), XelaSensorStamped, queue_size=1)
    self.cntr_pub = rospy.Publisher("~centered_{}".format(self._sensor_num), XelaSensorStamped, queue_size=1)
  
  def get_data_indices(self):
    # Index this sensor out of the full sensor data
    beg_index =  self._sensor_num    * taxel_num*3
    end_index = (self._sensor_num+1) * taxel_num*3
    return beg_index, end_index

  def update(self, data, base):
    beg_index, end_index = self.get_data_indices()
    self.base = base[beg_index:end_index]
    self.data = data[beg_index:end_index]
    self.cntr = [d - b for d, b in zip(self.data, self.base)]

  def calculate_forces(self, data):
    # Coefficient for sensor -> Newton is 25, taxel area is 0.0047**2
    force = np.array(data)
    f_x, f_y, f_z = (force.reshape((taxel_num,3))[:,ax] for ax in range(3))
    f_n=np.sum(f_z)
    f_t=[np.sum(f_x), np.sum(f_y)]
    center_of_pressure=[0,0]
    forces   = Forces('data', force, f_n, f_t, center_of_pressure)
    return forces
  
  def publish_reading(self, publisher, forces):
    msg = XelaSensorStamped()
    msg.board_id = self._board_id
    msg.header.stamp = rospy.Time.now()
    msg.data = forces.force
    if forces.type == 'data':
      msg.f_n = forces.f_n
      msg.f_t = forces.f_t
      msg.center_of_pressure = forces.center_of_pressure
    publisher.publish(msg)

  def publish(self):
    data = self.calculate_forces(self.data)
    cntr = self.calculate_forces(self.cntr)
    base = Forces('base', self.base)

    self.publish_reading(self.data_pub, data)
    self.publish_reading(self.base_pub, base) 
    self.publish_reading(self.cntr_pub, cntr) 


class XelaSensorArray(object):
  """
  This class is responsible for handling multiple sensors

  An array of sensors are stored, and the data for each is published during the main loop.
  As per the Xela Sensor documentation, up to 4 sensor controllers can be daisy chained,
  meaning that this class can handle up to 8 sensors over 4 controllers.

  This class handles data calibration, by starting an action service that can be requested.

  Attributes:
  board_id (int) = the board number for this array
  _sensor (obj)  = the sensor object, handles direct interface with the CAN to retrieve sensor readings
  sensors (list) = a list of all the SensorUnit objects handled by this array

  _calibrate_action_name (str) = the name of the calibrate action server
  _calibrate_action_server (obj) = the server itself
  _calibrate_action_result (obj) = the result of the calibration

  """

  def __init__(self):
    # Connect to the sensor and trigger to start acquisition
    board_id = rospy.get_param("board_id", 1) # Might have to change when more controllers are added
    self._sensor = XelaSensorInterface(board_id) # This too, might need an array of these
    self._sensor.start_data_acquisition()
    
    # Calibrate once
    data_dir = os.path.join(rospack.get_path("xela_ros"), "data")
    filename = os.path.join(data_dir, "log{}.csv".format(board_id))
    self._sensor.calibrate(sample_num=100, filename=filename)

    # Sensor data objects
    self._board_id = board_id
    self.sensors = [SensorUnit(board_id, sensor_num, self._sensor.base) for sensor_num in range(num_sensors)]

    # Calibrate action (for re-calibration)
    self._calibrate_action_name = "~calibrate"
    self._calibrate_action_server = actionlib.SimpleActionServer(self._calibrate_action_name, CalibrateAction, execute_cb=self.calibrate_action_callback, auto_start = False)
    self._calibrate_action_server.start()
    rospy.loginfo('Action server {} started.'.format(self._calibrate_action_name))
    self._calibrate_action_result = CalibrateResult()

  def calibrate_action_callback(self, goal):
    rospy.loginfo('Executing {}. request sent:'.format(self._calibrate_action_name))
    rospy.loginfo(goal)
    res = self._sensor.calibrate(sample_num=goal.sample_num, filename=goal.log_filename)
    self._calibrate_action_result.success = res
    self._calibrate_action_result.base = self._sensor.base
    self._calibrate_action_server.set_succeeded(self._calibrate_action_result)
    rospy.loginfo('Action server {} finished.'.format(self._calibrate_action_name))

  def update_sensor_data(self):
    # Get sensor data from sensor interface
    all_sensor_data = self._sensor.get_data(sensor_list)
    all_sensor_base = self._sensor.base
    # Update each sensor object, they retrieve their own data from list
    for sensor in self.sensors:
      sensor.update(all_sensor_data, all_sensor_base)

  def free_run(self):
    while not rospy.is_shutdown():
      self.update_sensor_data()
      for sensor in self.sensors: # Loop over sensors and publish their data
        sensor.publish()
      rospy.sleep(.001)


if __name__ == '__main__':
  # defect_sensor = [[int(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in ignore_sensor.rstrip(']').split(']')]
  rospy.init_node('xela_sensor', anonymous=True, log_level=rospy.DEBUG)
  node = XelaSensorArray()
  # Read the input sensor number, which are defect and create a list
  if len(rospy.get_param("~defect_sensor", None)) > 0:
    sensor_list = map(int, rospy.get_param("~defect_sensor", None).split(","))
    print("Selected sensor numbers, which are set to zero: ", sensor_list)
  else:
    sensor_list = None
  node.free_run()

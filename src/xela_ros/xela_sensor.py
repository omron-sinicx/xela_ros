# XelaSensor driver
# This class is ROS independent
import can
import csv
import numpy as np

# Base arbitration id for can interface
id_base     = 0x200
# Number of taxels (individual sensor nodes) on a sensor
taxel_num   = 16
# Number of sensors
num_sensors = 2

class XelaSensorInterface:
  """
  This class handles the direct communication with the Xela sensor controller (through CAN)

  Data is retrieved through "get_data", and returned as a list of 3*16 (3 axes for each taxel).
  Calibration is performed by averaging some readings from "get_data" to get the "base", which
  is also saved to disk.

  Attributes:
    _board_id (int) = board id for this sensor controller
    _bus (obj) = the can interface bus for interfacing with the sensor
    _base (list) = the most recent base (calibration reading) for this sensor
    _data (list) = the most recent data reading for this sensor

  """

  def __init__(self, board_id=1):
    self._board_id = board_id # board id is start from 1
    self._bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
    self._base = [0] * taxel_num*3 * num_sensors
    self._data = [0] * taxel_num*3 * num_sensors
    
  def __del__(self):
    self._bus.shutdown()

  def start_data_acquisition(self):
    # Trigger the microcontroller to return the data
    id = id_base|self._board_id # id of the first board is 0x201
    msg = can.Message(arbitration_id=id, data=[7,0], extended_id=False)
    try:
      self._bus.send(msg)
    except can.CanError:
      print("Message NOT sent")

  def get_data(self, sensor_list=None):
    try:
      while True:
        # Initialize empty data lists
        id_list   = [0] * taxel_num * num_sensors
        data_list = [0] * taxel_num * num_sensors
        success = False
        for i in range(0,taxel_num * num_sensors):
          recvmsg = self._bus.recv()
          # Data is located between id 256 and 256+taxel_num*num_sensors
          if (recvmsg.arbitration_id >= 256) and (recvmsg.arbitration_id < 256+taxel_num*num_sensors):
            id_list[recvmsg.arbitration_id - 256] = recvmsg.arbitration_id
            if recvmsg.data[1] < 256: # Changed this number from 250 to 256 to make the sensor run
              success = True
              data_list[recvmsg.arbitration_id - 256] = recvmsg.data
            else:
              success = False
              break
      
        if success:
          for i in range(taxel_num * num_sensors):
            # Update if data is available
            if id_list[i] != 0:
              # Combine MSB | LSB
              self._data[i*3+0] = data_list[i][1] << 8 | data_list[i][2]
              self._data[i*3+1] = data_list[i][3] << 8 | data_list[i][4]
              self._data[i*3+2] = data_list[i][5] << 8 | data_list[i][6]
              # Setting defect sensors to zero
              # if i == 4 or i == 7 or i == 9 or i == 10 or i == 15:
              if sensor_list:
                for x in sensor_list:
                  if x == i:
                    self._data[i*3+0] = 0
                    self._data[i*3+1] = 0
                    self._data[i*3+2] = 0
          break
    except KeyboardInterrupt:
      return

    return self._data

  def calibrate(self, sample_num=100, filename="log.csv"):
    """
    This function calibrates the sensor, by taking sample_num number of readings using "get_data"

    Data from these readings is averaged, saved to a file, and stored in the class attribute _base.
    This can be used later to center the data readings to this calibration reading.

    Args:
      sample_num (int) = number of samples to average over when calibrating
      filename (str) = the name for the file to save the calibration reading

    """
    with open(filename, 'wb') as f:
      writer = csv.writer(f, delimiter=',')
      writer.writerow(['B1S1X','B1S1Y','B1S1Z','B1S2X','B1S2Y','B1S2Z','B1S3X','B1S3Y','B1S3Z','B1S4X','B1S4Y','B1S4Z',
                       'B2S1X','B2S1Y','B2S1Z','B2S2X','B2S2Y','B2S2Z','B2S3X','B2S3Y','B2S3Z','B2S4X','B2S4Y','B2S4Z',
                       'B3S1X','B3S1Y','B3S1Z','B3S2X','B3S2Y','B3S2Z','B3S3X','B3S3Y','B3S3Z','B3S4X','B3S4Y','B3S4Z',
                       'B4S1X','B4S1Y','B4S1Z','B4S2X','B4S2Y','B4S2Z','B4S3X','B4S3Y','B4S3Z','B4S4X','B4S4Y','B4S4Z'])
      base = np.zeros([taxel_num*3 * num_sensors])
      for k in range(sample_num):
        data = self.get_data()
        writer.writerow(data)
        base = base + np.array(data)
      self._base = (base / sample_num).tolist()
    return True

  @property
  def data(self):
    return self._data

  @property
  def base(self):
    return self._base

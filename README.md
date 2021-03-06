# Introduction

This package provides sensor data from the Xela robotics tactile sensor with 16 taxels.
The sensor node publishes the XelaSensorStamped message. 
The sensor data is in the "data" member of the message as a float array [x1,y1,z1,x2,y2,z2,...,x16,y16,z16]

The "calibrate" action is available to re-zero the sensor.

# Initial Setup
* Install requirements

    ```bash
    pip install opencv-python
    pip install python-can
    ```

# How to use

1. Connect the sensor to the PC.
2. Before running, do:

    ```bash
    sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0
    ```

3. Run demonstration to visualize the states of taxels as circles.

    ```
    roslaunch xela_ros demo.launch
    ```

In order to start the XELA sensor without visualization and set sensor values to zero, sensor.launch needs to be executed like this:
    ```
    roslaunch xela_ros sensor.launch
    ```

# Multiple sensors

The code assumes that two sensors are connected to one sensor controller. To use a single sensor, change the number of sensors [here](https://github.com/omron-sinicx/xela_ros/blob/master/src/xela_ros/xela_sensor.py#L12). To use more daisy-chained sensors and controllers, you will need to extend the code slightly [here](https://github.com/omron-sinicx/xela_ros/blob/master/scripts/xela_sensor_node.py#L106). Pull requests are welcome.

# ROS python node publishing a serial joystick to /joy topic 

ROS node to interface a joystick accessed through serial device /cmd_vel robot control

This node polls joystick settings and buttons over a serial port

It is by default setup with a simple protocol for the Mark-Toys.com  Multi-Port USB card which can amoung other things read the 2 potentiometers on an analog joystick and be queried for state of the X and Y settings.


This node then publishes to either (or both) a /joy or /cmd_vel type of ROS topic
The /joy topic gets joystick messages and the /cmd_vel topic gets 'twist' messages
in the future buttons will be mapped to the /joy 'buttons' array as 1 for pressed and 0 for released
The default /cmd_vel topic and speeds may be changed using parameters
Release of the joystick causes velocity in all modes to go back to 0

Configuration is done in:  param/serial_joystick.yaml
The serial device must use the protocol coded into this node.

This node can thus be modified to access other forms of serial joystick devices with little effort.

# ROS parameters saved in the yaml config file in this folder

    ROS Param            Default  Description
    serial_device           xx    Serial device such as /dev/ttyUSB0
    output_to_joy           1     Output control messages to the /joy ROS topic using Joy messages
    output_to_cmd_vel       0     Output control messages to the /cmd_vel ROS topic using twist messages
    speed_stopped           0.0   Robot X speed when stopped (can be used to null out robot offset)
    speed_fwd_normal        0.2   Robot forward X speed in M/Sec for joystick forward straight
    speed_fwd_inc           0.05  Increment in M/Sec for a speed increment
    speed_fwd_turning       0.2   Robot forward X speed in M/Sec for joystick in turn mode (not rotate mode)
    speed_rev_normal       -0.15  Robot reverse X speed in M/Sec for joystick reverse straight back
    angular_straight_rate   0.0   Robot Z angular rate when stopped (can be used to null out robot offset)
    angular_turning_rate    0.3   Robot Z angular rate in Rad/Sec when turning (not for rotate)
    angular_rotate_rate     0.4   Robot Z angular rate in Rad/Sec when rotating
    angular_rotate_inc      0.1   Increment in ad/Sec for a rotate increment
    angular_rotate_max      2.0   Max Robot Z angular rate in Rad/Sec

# Install This Node

    cd ~/catkin_ws/src
    git clone https://github.com/mjstn/serial_joystick.git
    catkin_make

# To run this node when the ROS master node is running

    roslaunch serial_joystick serial_joystick_node.launch

Optionally you can then include the launch in a ROS launch file as a node if you are aware of how that is done.

    
Run the raw python code without full launch file
Plug in and power up the joystick device and serial port used to talk to the joystick.
Next run the node from catkin_ws:   catkin_ws> roslaunch serial_joystick serial_joystick.launch
To run the node without ROS launch or the yaml config file you can run from where the script is:
python nodes/serial_joystick.py /dev/ttyUSB0

# The default serial protocol
A very simple protocol is done to talk to a custom Mark-Toys.com board I call the USB Multi-Port .
Because the USB to serial is on this small dongle board and the M0 processor that reads the analog lines is less than a centimeter away I have not implemented any checksums or packets to get joystick over serial.  If this code is modified to use some custom joystick that goes over real wires to some other little processor I strongly suggest you worry about the integrity of the bits.  

The custom ARM M0 on the USB Multi-Port board requires this node send  :V;   where : means a command is starting. V is the command to read back two voltages in a simple    xxx,yyy form both integers from 0-4095.  The code subtracts off 2048 and presto the code has a signed integer.  


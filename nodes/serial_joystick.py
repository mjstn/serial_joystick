#!/usr/bin/python -u

#
# ROS node to interface a joystick accessed through serial device /cmd_vel robot control
#
# Thes node polls joystick settings and buttons over a serial port
#
# This node then publishes to either (or both) a /joy or /cmd_vel type of ROS topic
# The /joy topic gets joystick messages and the /cmd_vel topic gets 'twist' messages
# in the future buttons will be mapped to the /joy 'buttons' array as 1 for pressed and 0 for released
# The default /cmd_vel topic and speeds may be changed using parameters
# Release of the joystick causes velocity in all modes to go back to 0
#
# Configuration is done in:  catkin_ws/src/demos/serial_joystick/serial_joystick.yaml
# The serial device must use the protocol coded into this node.
#
# This node can thus be modified to access other forms of serial joystick devices with little effort.
#
# ROS Param            Default  Description
# serial_device           xx    Serial device such as /dev/ttyUSB0
# output_to_joy           1     Output control messages to the /joy ROS topic using Joy messages
# output_to_cmd_vel       0     Output control messages to the /cmd_vel ROS topic using twist messages
# speed_stopped           0.0   Robot X speed when stopped (can be used to null out robot offset)
# speed_fwd_normal        0.2   Robot forward X speed in M/Sec for joystick forward straight
# speed_fwd_inc           0.05  Increment in M/Sec for a speed increment
# speed_fwd_turning       0.2   Robot forward X speed in M/Sec for joystick in turn mode (not rotate mode)
# speed_rev_normal       -0.15  Robot reverse X speed in M/Sec for joystick reverse straight back
# angular_straight_rate   0.0   Robot Z angular rate when stopped (can be used to null out robot offset)
# angular_turning_rate    0.3   Robot Z angular rate in Rad/Sec when turning (not for rotate)
# angular_rotate_rate     0.4   Robot Z angular rate in Rad/Sec when rotating
# angular_rotate_inc      0.1   Increment in ad/Sec for a rotate increment
# angular_rotate_max      2.0   Max Robot Z angular rate in Rad/Sec

# Run the raw python code without full launch file
# Plug in and power up the joystick device and serial port used to talk to the joystick.
# Next run the node from catkin_ws:   catkin_ws> roslaunch serial_joystick serial_joystick.launch
# To run the node without ROS launch or the yaml config file you can run from where the script is:
# python nodes/serial_joystick.py /dev/ttyUSB0
#
# System Limitations (beside having ROS environment and bt_joystick node configured)
# - Must have the serial device set in serial_joystick.yaml 
#
# Development Info
# g_debug can enable ros logs or debug prints but do NOT ship in that state as it's flakey
#
#
# History:
# 20190116       Formed this node from bt_joystick node
#

from __future__ import print_function

import sys
import time
import serial
import string
from threading import Event
from threading import Thread

# import ROS related support including Twist messages
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# simple version string
g_version = "20190120"

# Bluetooth MAC address if none is supplied
g_serial_device = "/dev/ttyUSB1"

# Define a string that will query the joystick for bat-handle setting
# It is expected that the reply will be a simple  xValue,yValue pair
# The intitial joystick we use is fully USB so there is no need for checksums and so on.
g_joystick_query  = ":V;"    

# define offset and scale so we get ROS value of  9ADC_Count - offset) / scale
g_joystick_offset  = 2048
g_x_joystick_scale = 2050.0
g_y_joystick_scale = 2050.0

# This debug flag if set True enables prints and so on but cannot be used in production
g_debug = False

# Workaround for disconnect returning prior to truely being disconnected is an ugly delay
g_disconnectTime = float(0.4)

g_connectSuccessful = int(0)

# Button and Joystick bytes
g_joystickBits = int(0)
g_buttonBits   = int(0)
g_fwdVel       = float(0.0)

# define bits we read from ring controller for keys pressed
button_a_bit   = int(4)   # 0x10
button_b_bit   = int(0)   # 0x01
button_c_bit   = int(3)   # 0x08
button_d_bit   = int(1)   # 0x02
button_o_bit   = int(6)   # 0x40
button_r_bit   = int(7)   # 0x80

# define what button bit is set in /joy message for each BT joystick button
button_a_joy_bit   = int(0)   # 0x10
button_b_joy_bit   = int(1)   # 0x01

# define which keys will be used to increment speed for max joystick values
g_speed_increase_bit = int(button_a_bit)
g_speed_decrease_bit = int(button_c_bit)

# Simple wrappers to prevent log overhead and use desired log functions for your system
def logAlways(message):
    rospy.loginfo(message)

def logDebug(message):
    global g_debug
    if g_debug == True:
        rospy.loginfo(message)


# test a single bit of an integer to see if it is set or clear
def testBit(int_type, offset):
    mask = 1 << offset
    return(int_type & mask)

class serialjoystick():
    def __init__(self):
        
        # The bits we see here are placed in globals by notification callback
        global g_serial_device
        global g_joystickBits
        global g_buttonBits

        g_joystickBits  = 1
        g_buttonBits    = 1
        oldJoystickBits = -1
        oldButtonBits   = -1

        logAlways("Starting joystick thread that publishes twist to ROS")
    
        # initiliaze
        rospy.init_node('serial_joystick', anonymous=False)

        # The bit codes for the joystick movement
        # User MUST hit M and B to get to gamepad mode!
        joyFwdBits          = int('0x10',16)
        joyFwdRightBits     = int('0x20',16)
        joyRotRightBits     = int('0x60',16)
        joyFwdLeftBits      = int('0x00',16)
        joyRotLeftBits      = int('0x40',16)
        joyRevBits          = int('0x90',16)

        logAlways("Setup Node Parameters ...");
        # Now adjust the parameters from ros config params if they exist
        # Comment this out to not worry about config but if used btjoystick.cfg will set the defaults
        self.serial_device = rospy.get_param("~serial_device", g_serial_device)
        g_serial_device = self.serial_device

        # Define which sort of outputs this node will send to ROS topics for bot control
        self.output_to_joy = rospy.get_param("~output_to_joy", 1)
        self.output_to_cmd_vel = rospy.get_param("~output_to_cmd_vel", 0)

        # Best to keep fwd_max / fwd_inc integer and equal to rev_max / rev_inc
        self.speed_fwd_max = rospy.get_param("~speed_fwd_max", 2.0)
        self.speed_fwd_inc = rospy.get_param("~speed_fwd_inc", 0.1)
        self.speed_rev_max = rospy.get_param("~speed_fwd_max", -1.0)
        self.speed_rev_inc = rospy.get_param("~speed_fwd_inc", 0.05)
        self.speed_stopped = rospy.get_param("~speed_stopped", 0.0)
        self.speed_fwd_normal = rospy.get_param("~speed_fwd_normal", 0.3)
        self.speed_fwd_turning = rospy.get_param("~speed_fwd_turning", 0.2)
        self.speed_rev_normal = rospy.get_param("~speed_rev_normal", -0.15)
        self.speed_rev_inc = rospy.get_param("~speed_fwd_inc", 0.05)
        self.angular_straight_rate = rospy.get_param("~angular_straight_rate", 0.0)
        self.angular_turning_rate = rospy.get_param("~angular_turning_rate", 0.3)
        self.angular_rotate_rate = rospy.get_param("~angular_rotate_rate", 1.5)
        self.angular_rotate_inc = rospy.get_param("~angular_rotate_inc", 0.1)
        self.angular_rotate_max = rospy.get_param("~angular_rotate_inc", 3.0)

        logAlways("Node Parameters:")
        logAlways("Dev: " + self.serial_device)
        logAlways("Fwd: " + str(self.speed_fwd_normal) + " FwdTurning: " + str(self.speed_fwd_turning) + " TurboMult: ")
        logAlways("AngTurningRadSec: " + str(self.angular_turning_rate) + " AngRotateRadSec: " + str(self.angular_rotate_rate))
        if self.output_to_cmd_vel == 1:
            logAlways("ROS twist messages will appear on ROS topic: /cmd_vel_joy")
        if self.output_to_joy == 1:
            logAlways("ROS joystick messages will appear on ROS topic: /joy")

        #  tell user how to stop TurtleBot
        logAlways("To stop the BT Joystick node use CTRL + Z")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # Setup ROS publisher(s) depending on what outputs you have enabled for this node
        # Setup the joy message if joy ROS output is to be used
        joy_msg = Joy()

        #if self.output_to_joy == 1:
        #    # Create a publisher which can "talk" to the ROS /joy topic and act like a joystick
        #    self.joy = rospy.Publisher('/joy', Joy, queue_size=10)
        self.joy = rospy.Publisher('/joy', Joy, queue_size=10)

        # Setup the Twist message if cmd_vel ROS output is to be used
        move_cmd = Twist()
        #if self.output_to_cmd_vel == 1:
        #    #  Create a publisher which can "talk" to TurtleBot and tell it to move
        #    # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        #    self.cmd_vel = rospy.Publisher('/cmd_vel_joy', Twist, queue_size=10)
        self.cmd_vel = rospy.Publisher('/cmd_vel_joy', Twist, queue_size=10)
        
        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        
        #  let's set velocity to 0 at this time
        move_cmd.linear.x = self.speed_stopped
        #  let's turn at 0 radians/s
        move_cmd.angular.z = self.speed_stopped

        joy_msg.buttons = [int(0), int(0), int(0), int(0), int(0), int(0)]
        joy_msg.axes = [0.0, 0.0]

        # Open the serial device or die now with error
        print("Start Serial port using device ", g_serial_device)
        ser = serial.Serial(g_serial_device, 38400, 8, 'N', 1, timeout=1)
        logAlways("Serial port started")
        # ser.open()
        # logAlways("Serial port opened")

        if rospy.is_shutdown():
            logAlways("ROS appears to be shutdown. Run roscore")

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # We do not force linear and angular speeds to 0 unless joystick is not pressed
            # move_cmd.linear.x = 0.0
            # move_cmd.angular.z = 0.0
            # Show when the bits have changed

            # This node uses an extremely simple poll then publish mode of operation
            cmdPacket = g_joystick_query
            ser.write(cmdPacket)

            replyPacket = ser.flushInput()
            replyPacket = ser.readline()

            # For debug use this:    logDebug("Joystick X,Y: " + str(replyPacket))

            # parse off the 2 values
            xypair = replyPacket.split(",")
            logDebug("Joystick X,Y values: " + xypair[0] + ":" + xypair[1])
            xJoyAdc = xypair[0]
            yJoyAdc = xypair[1]

            # Now calculate a signed -1.0 to 1.0 value for each joystick axis
            xJoyNormalized = (int(str(xJoyAdc)) - int(str(g_joystick_offset))) / float(str(g_x_joystick_scale))
            yJoyNormalized = (int(str(yJoyAdc)) - int(str(g_joystick_offset))) / float(str(g_y_joystick_scale))
            
            # Convert the numeric values received to linear X and angular Z 
            move_cmd.linear.x  =  xJoyNormalized * self.speed_fwd_max
            move_cmd.angular.z =  yJoyNormalized * self.angular_rotate_max

             # publish the joystick output messages
            if self.output_to_joy == 1:
                # convert linear and angular speeds into joystick values from 0-1 based on max rates
                joy_linear  = move_cmd.linear.x / self.speed_fwd_max
                joy_angular = move_cmd.angular.z / self.angular_rotate_max
                logDebug("Publish Joystick speeds: angular " + str(joy_angular) + " linear: " + str(joy_linear))
                joy_msg.axes = [joy_angular, joy_linear]

                # pack rhe button bits
                self.joy.publish(joy_msg)

            # publish the velocity
            if self.output_to_cmd_vel == 1:
                self.cmd_vel.publish(move_cmd)

            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
        logAlways("ROS HAS SHUTDOWN!")

    def cfg_callback(self, config, level):
        self.config = config
        return config

    def shutdown(self):
        # stop turtlebot
        logAlways("SHUTDOWN! Stopping this node")
        # ser.close()
        sys.exit(1)
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    print("Running with version: " + g_version)
 
    try:
        serialjoystick()
    except RuntimeError,e: 
        logAlways("Exception in serial_joystick: " + e.message)
    except Exception:
        logAlways("serial_joystick node terminated.")

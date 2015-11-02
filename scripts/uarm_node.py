#!/usr/bin/env python

import sys
import serial
import time
import binascii
import struct

import numpy as np

import rospy
# import roslib

from sensor_msgs.msg import JointState

dev_port = '/dev/ttyUSB0'
baud_rate=9600

ser_global = None

sync_token = '#SYNC\n'

rotAngle = 0.0
stretchAngle = 0.0
heightAngle = 0.0
handRotAngle = 0.0
handClose = 0.0

rotAngleCmd     = 0
stretchAngleCmd = 0
heightAngleCmd  = -np.radians(15)
handRotAngleCmd = 0
handCloseCmd    = 2 

state_topic = '/uarm_node/joint_state'
cmd_topic = '/uarm_node/joint_cmd'

bSync = False

rotAngleLimit       = [-90, 90]
stretchAngleLimit   = [0, 210]
heightAngleLimit    = [-180, 150]
handRotAngleLimit   = [-90, 90]

def truncate_angle(angle, limit):
    ang_truncated = max([limit[0], angle])
    ang_truncated = min([limit[1], ang_truncated])
    return ang_truncated

def read_float(serial):
    """
    helper function to read a float from the serial port
    """
    #convert from little endian to big endian
    hex_array = reversed([chr(ord(serial.read(1))) for i in range(4)])
    # hex_array = [chr(ord(serial.read(1))) for i in range(4)]
    hex_string = ''.join(hex_array)
    return struct.unpack('>f', hex_string)[0]

def pack_float(value_in):
    """
    helper function to convert a float to the binary form
    """
    bin = struct.pack('<f', value_in)
    return 

def pack_short(value_in):
    """
    helper function to convert a short to the binary form
    note the bit operation is already included
    """
    return struct.pack('!h', value_in)

def pack_byte(value_in):
    """
    helper function to convert bool to the bnary form
    """
    return struct.pack('B', value_in)

def write_cmd_header():
    ser_global.write('\xFF')
    ser_global.write('\xAA')
    return

def write_cmd():
    global ser_global
    global rotAngleCmd, stretchAngleCmd, heightAngleCmd, handRotAngleCmd
    global rotAngleLimit, stretchAngleLimit, heightAngleLimit, handRotAngleLimit
    #rotation
    rotCmd = int(truncate_angle(np.degrees(rotAngleCmd), rotAngleLimit))
    stretchCmd = int(truncate_angle(np.degrees(stretchAngleCmd), stretchAngleLimit))
    heightCmd = int(truncate_angle(np.degrees(heightAngleCmd), heightAngleLimit))
    handRotCmd = int(truncate_angle(np.degrees(handRotAngleCmd), handRotAngleLimit))

    handOpenCmd = 2
    #constant for now
    handOpenCmd = False
    print 'rotCmd:', rotCmd, 'stretchCmd:', stretchCmd, 'heightCmd:', heightCmd, 'handRotCmd:', handRotCmd

    write_cmd_header()
    ser_global.write(pack_short(rotCmd))
    ser_global.write(pack_short(stretchCmd))
    ser_global.write(pack_short(heightCmd))
    ser_global.write(pack_short(handRotCmd))
    ser_global.write(pack_byte(handOpenCmd))
    return

def uarm_joint_cmd_callback(joint_cmd):
    #<WARNING> there might be synchronization issue for accessing the global variables
    #race condition might exist with write_cmd, that's just because rospy use a different mechanism and there is no spinOnce()...
    #probably it would be better to re-write this with C++..
    global rotAngleCmd, stretchAngleCmd, heightAngleCmd, handRotAngleCmd
    rotAngleCmd         = joint_cmd.position[0]
    stretchAngleCmd     = joint_cmd.position[1]
    heightAngleCmd      = joint_cmd.position[2]
    handRotAngleCmd     = joint_cmd.position[3]
    handCloseCmd        = joint_cmd.position[4]
    return

def dummay_joint_cmd_test():
    dummy_cmd_msg = JointState()
    dummy_cmd_msg.position = [np.radians(0), np.radians(105), np.radians(-15), np.radians(0), 0]
    uarm_joint_cmd_callback(dummy_cmd_msg)

    return

def main(rate=50):
    global bSync
    global sync_token
    global rotAngle, stretchAngle, heightAngle, handRotAngle, handClose
    global state_topic, cmd_topic
    global ser_global

    rospy.init_node('uarm_node', anonymous=True)

    #<TODO> use ros parameter server to configure the settings...
    ser = serial.Serial(dev_port, baud_rate)
    ser_global = ser

    pub = rospy.Publisher(state_topic, JointState, queue_size=10)
    sub = rospy.Subscriber(cmd_topic, JointState, uarm_joint_cmd_callback)

    rate = rospy.Rate(rate)

    state_msg = JointState()

    # tmpCnt = 0

    while not rospy.is_shutdown():
        if not bSync:
            rospy.loginfo('UARM_NODE: Trying to setup and synch uArm...')
            #wait 3 seconds
            time.sleep(3)
            ser.write(sync_token)

            #request a sync token
            ser.flushInput()
            ser.flushOutput()

            bMatch = False
            bMiss = False

            while not bMatch:
                #check the serial port
                if ser.inWaiting() < len(sync_token):
                    continue
                else:
                    #now check the reading against token
                    for i in range(len(sync_token)):
                        if ser.read(1) != sync_token[i]:
                            bMiss = True
                            break
                    if bMiss:
                        bMiss = False   #need a recheck...
                    else:
                        bMatch = True
            bSync = True

            rospy.loginfo('UARM_NODE: Initialization done. Running...')
        else:
            write_cmd()
            #now let's read the joint states
            #20 bytes: 16 bytes for 4DOFs and 4 bytes for the hand close
            if ser.inWaiting() >= 20:
                rotAngle = np.radians(read_float(ser))
                stretchAngle = np.radians(read_float(ser))
                heightAngle = np.radians(read_float(ser))
                handRotAngle = np.radians(read_float(ser))
                handClose = np.radians(read_float(ser))

                # print 'rotState:', rotAngle, stretchAngle, heightAngle, handRotAngle

                state_msg.position = [rotAngle, stretchAngle, heightAngle, handRotAngle]
                state_msg.header.stamp = rospy.Time.now()

                pub.publish(state_msg)

            #construct a dummy command message to test
            # if tmpCnt > 100:
            #     tmpCnt = 0
            #     dummay_joint_cmd_test()
            # else:
            #     tmpCnt += 1

        rate.sleep()

    return


if __name__ == '__main__':
    main()
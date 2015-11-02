#!/usr/bin/env python
"""
A test to the uarm_node, test by sending sinuoid joint trajectory
"""
import numpy as np

import rospy
import time
# import roslib

from sensor_msgs.msg import JointState


bSync = False

rotAngle        = 0
stretchAngle    = 0
heightAngle     = 0
handRotAngle    = 0

#this is not perfectly safe as we might have problem with the synchronization of the variables...
def joint_state_callback(msg):
    global bSync
    global rotAngle, stretchAngle, heightAngle, handRotAngle

    if not bSync:
        bSync = True
        rotAngle        = msg.position[0]
        stretchAngle    = msg.position[1]
        heightAngle     = msg.position[2]
        handRotAngle    = msg.position[3]
    return

def main():
    global bSync
    global rotAngle, stretchAngle, heightAngle, handRotAngle

    rospy.init_node('uarm_node_test', anonymous=True)

    rate = rospy.Rate(20)
    #prepare a sinuoid trajectory for joints

    curr_pos = None

    state_topic = '/uarm_node/joint_state'
    cmd_topic = '/uarm_node/joint_cmd'

    pub = rospy.Publisher(cmd_topic, JointState, queue_size=10)
    sub = rospy.Subscriber(state_topic, JointState, joint_state_callback)

    amplitude = np.radians(60)
    cmd_msg = JointState()
    idx = 0
    
    bSync = True

    while not rospy.is_shutdown():
        if not bSync:
            rospy.loginfo('UARM_TEST_NODE: Trying to synch the sensor reading...')
            time.sleep(3)
        else:
            if idx > 50:
                idx = 0
            
            stretchCmd = 0*stretchAngle + amplitude * np.abs(np.sin(2*np.pi*float(idx)/50))
            heightCmd = heightAngle + amplitude * 0.5 *np.sin(2*np.pi*float(idx)/50)
            # cmd_msg.position = [0, stretchCmd, 0, 0, 2]
            cmd_msg.position = [0, 0, heightCmd, 0, 2]
            # cmd_msg.position = [0, stretchCmd, heightCmd, 0, 2]
            idx += 1

            pub.publish(cmd_msg)
        rate.sleep()

    return


if __name__ == '__main__':
    main()
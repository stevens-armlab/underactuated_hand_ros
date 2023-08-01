#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from hand_pkg.driver import hand_driver
hand = hand_driver()

def joy_cb(data):
    global hand
    if data.buttons[5]: #RB on controller
        hand.grasp(speed=60, pos_max=6400)
    elif data.buttons[4]: #LB on controller
        hand.ungrasp(position = -400) #set home position here
    else:
        hand.stop_grasp()

    if data.buttons[1]:  #B button
        hand.start_spread()
    elif data.buttons[2]:  #X button
        hand.start_unspread()
    else:
        hand.stop_spread()

if __name__ == '__main__':
    rospy.init_node('hand_teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
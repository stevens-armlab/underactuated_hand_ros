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
        hand.grasp(speed=40, pos_max=5800)
    elif data.buttons[4]: #LB on controller
        hand.ungrasp(position = 2700)
    else:
        hand.stop_grasp()

    if data.buttons[2]:
        hand.start_spread()
    elif data.buttons[3]:
        hand.start_unspread()
    else:
        hand.stop_spread()

if __name__ == '__main__':
    rospy.init_node('hand_teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
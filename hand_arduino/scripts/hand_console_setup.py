#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from hand_pkg.driver import hand_driver
import numpy as np

if __name__ == '__main__':
    rospy.init_node('hand_console', anonymous=True)
    hand = hand_driver()
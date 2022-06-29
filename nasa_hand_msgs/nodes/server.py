#!/usr/bin/env python
#To run: 'rosrun nasa_hand_msgs server.py'

import rospy

from dynamic_reconfigure.server import Server
from nasa_hand_msgs.cfg import pot_minsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {pot1_min}, {pot2_min}, {pot3_min},\
    {pot4_min}, {pot5_min}, {pot6_min}, {pot7_min}, {pot8_min},""".format(**config))
    return config

if __name__ == "__main__":
#Name of the reconfigurable node:
    rospy.init_node("pot_mins_node", anonymous = False)

    srv = Server(pot_minsConfig, callback)
    rospy.spin()

#!/usr/bin/env python
#Included in launch file so runs automatically
#To run manually: 'rosrun nasa_hand_msgs server.py'

import rospy

from dynamic_reconfigure.server import Server
from hand_rviz.cfg import PotMinsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {pot0}, {pot1}, {pot2}, {pot3},\
    {pot4}, {pot5}""".format(**config))
    return config

if __name__ == "__main__":
#Name of the reconfigurable node:
    rospy.init_node("pot_mins", anonymous = False)

    srv = Server(PotMinsConfig, callback)
    rospy.spin()

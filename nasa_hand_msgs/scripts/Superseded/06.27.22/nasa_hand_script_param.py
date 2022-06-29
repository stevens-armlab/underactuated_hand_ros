#!/usr/bin/env python

#HOW TO USE THIS SCRIPT:
#Run 'roslaunch nasa_hand_urdf display.launch' to open rviz
#Run 'rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600' to connect to Teensy
#Run 'rosrun nasa_hand_msgs nasa_hand_script_param.py'

import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nasa_hand_msgs.msg import nasaHandPots

#In display.launch, must add '<rosparam param="source_list">["/joint_states_command"]</rosparam>'
#to joint_state_publisher_gui node

print(r"""
  _   _    _    ____    _      _   _    _    _   _ ____
 | \ | |  / \  / ___|  / \    | | | |  / \  | \ | |  _ \
 |  \| | / _ \ \___ \ / _ \   | |_| | / _ \ |  \| | | | |
 | |\  |/ ___ \ ___) / ___ \  |  _  |/ ___ \| |\  | |_| |
 |_| \_/_/   \_\____/_/   \_\ |_| |_/_/   \_\_| \_|____/

""")

print "Please ensure hand is fully extended."
raw_input("Press ENTER to continue...")
#Name node hand_sim
rospy.init_node('hand_sim', anonymous=True)

def joint_message():
    message = JointState()
    #message.position = [pot7, pot1, pot2, pot8, pot3, pot4, pot5, pot6]
    #Since we only have two potentiometers installed right now:
    message.position = [0, pot1, pot2, 0, 0, pot3, 0, 0]
    message.name = ["finger1_roll_joint", "finger1_prox_joint", "finger1_dist_joint", "finger2_roll_joint", "finger2_prox_joint", "finger2_dist_joint", "thumb_prox_joint", "thumb_dist_joint"]

    joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)
    joint_state_pub.publish(message)

count = 0

#Assign ros params (from yaml file) to variables
potrange = rospy.get_param('/range')
mins = rospy.get_param('/mins')

def callback(data):
    #Initialize variables:
    global pot1, pot2, pot3, pot4, pot5, pot6, pot7, pot8
    global pot1_min, pot2_min, pot3_min, pot4_min, pot5_min, pot6_min, pot7_min, pot8_min

    """
    #Capture initial position of potentiometers and save as minimum values:
    global count
    while True:
        if (count==1):
            break
        else:
            rospy.set_param('/mins/pot1', data.pot1)
            rospy.set_param('/mins/pot2', data.pot2)
            rospy.set_param('/mins/pot3', data.pot3)
            rospy.set_param('/mins/pot4', data.pot4)
            rospy.set_param('/mins/pot5', data.pot5)
            rospy.set_param('/mins/pot6', data.pot6)
            rospy.set_param('/mins/pot7', data.pot7)
            rospy.set_param('/mins/pot8', data.pot8)
            count = 1
    """

    #Range to normalize data to: [a, b]
    a = 0
    b = 1.57

    #To scale variable x into range [a, b]: x_scaled = (b-a)((x-min(x))/(max(x)-min(x))+a
    pot1 = -b*((data.pot1 - mins['pot1'])/(potrange))
    pot2 = -b*((data.pot2 - mins['pot2'])/(potrange))
    pot3 = -b*((data.pot3 - mins['pot3'])/(potrange))
    pot4 = -b*((data.pot4 - mins['pot4'])/(potrange))
    pot5 = -b*((data.pot5 - mins['pot5'])/(potrange))
    pot6 = -b*((data.pot6 - mins['pot6'])/(potrange))
    pot7 = -b*((data.pot7 - mins['pot7'])/(potrange))
    pot8 = -b*((data.pot8 - mins['pot8'])/(potrange))

    pot_vals = [pot1, pot2, pot3, pot4, pot5, pot6]
    for x in range (0,5):
        print pot_vals[x]

    print ""

    #Execute joint_message function above
    joint_message()

def listener():
    #Node subscribes to potpub topic which is of type nasaHandPots. When new messages are received,
    #callback is invoked with the message as the first argument.
    rospy.Subscriber('pot_pub', nasaHandPots, callback)
    rospy.spin()

if __name__ == '__main__':

    listener()

#Prevents the buffer from being overloaded with messages
while not rospy.is_shutdown():
    	rate.sleep()

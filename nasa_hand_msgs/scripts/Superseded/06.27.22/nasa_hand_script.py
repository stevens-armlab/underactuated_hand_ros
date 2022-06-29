#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nasa_hand_msgs.msg import nasaHandPots

#In display.launch, must add '<rosparam param="source_list">["/joint_states_command"]</rosparam>'
#to joint_state_publisher_gui node
#Retrieves data from subscribed topic (pubimu)


#Name node hand_sim
rospy.init_node('hand_sim', anonymous=True)


def joint_message():
    message = JointState()
    #message.position = [pot7, pot1, pot2, pot8, pot3, pot4, pot5, pot6]
    #Since we only have two potentiometers installed right now:
    message.position = [0, pot1, pot2, 0, 0, 0, 0, 0]
    message.name = ["finger1_roll_joint", "finger1_prox_joint", "finger1_dist_joint", "finger2_roll_joint", "finger2_prox_joint", "finger2_dist_joint", "thumb_prox_joint", "thumb_dist_joint"]

    joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)

    joint_state_pub.publish(message)

count = 0

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
            pot1_min = data.pot1
            pot2_min = data.pot2
            pot3_min = data.pot3
            pot4_min = data.pot4
            pot5_min = data.pot5
            pot6_min = data.pot6
            pot7_min = data.pot7
            pot8_min = data.pot8
            count = 1
    """
    #Need to manually specifiy min value of each sensor:
    pot1_min = 240.987
    pot2_min = 349.600
    pot3_min = 0
    pot4_min = 0
    pot5_min = 0
    pot6_min = 0
    pot7_min = 0
    pot8_min = 0

    #Need to manually specifiy range (max - min) of each sensor:
    pot1_range = 211.146
    pot2_range = 211.999
    pot3_range = 363
    pot4_range = 303
    pot5_range = 363
    pot6_range = 303
    pot7_range = 100
    pot8_range = 100

    #Range to normalize data to: [a, b]
    a = 0
    b = 1.57

    #To scale variable x into range [a, b]: x_scaled = (b-a)((x-min(x))/(max(x)-min(x))+a
    pot1 = -b*((data.pot1 - pot1_min)/(pot1_range))
    pot2 = -b*((data.pot2 - pot2_min)/(pot2_range))
    pot3 = -b*((data.pot3 - pot3_min)/(pot3_range))
    pot4 = -b*((data.pot4 - pot4_min)/(pot4_range))
    pot5 = -b*((data.pot5 - pot5_min)/(pot5_range))
    pot6 = -b*((data.pot6 - pot6_min)/(pot6_range))
    pot7 = -b*((data.pot7 - pot7_min)/(pot7_range))
    pot8 = -b*((data.pot8 - pot8_min)/(pot8_range))

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

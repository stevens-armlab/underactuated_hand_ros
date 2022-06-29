#!/usr/bin/env python

#HOW TO USE THIS SCRIPT:
#Make sure you source before each of the following commands: 'source ~/catkin_ws/devel/setup.bash'
#Run 'roslaunch nasa_hand_urdf display.launch' to open rviz
#Run 'rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600' to connect to Teensy
#Run 'rosrun nasa_hand_msgs server.py' to set up dynamic parameters
#Navigate to this file's location and run './nasa_hand_script_param.py'

import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nasa_hand_msgs.msg import nasaHandPots
#For dynamic parameters:
import dynamic_reconfigure.client

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

#For dynamic reconfigure, see server.py
client = dynamic_reconfigure.client.Client('pot_mins_node')

def joint_message():
    message = JointState()
    #message.position = [pot7, pot1, pot2, pot8, pot3, pot4, pot5, pot6]
    #Some have negative signs to account for the sensors reversing in the finger:
    message.position = [0, pot1, pot2, 0, pot3, -pot4, pot5, pot6]
    message.name = ["finger1_roll_joint", "finger1_prox_joint", "finger1_dist_joint", "finger2_roll_joint", "finger2_prox_joint", "finger2_dist_joint", "thumb_prox_joint", "thumb_dist_joint"]

    joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)
    joint_state_pub.publish(message)

count = 0

#Assign ros params (from yaml file) to variables
#Must restart display.launch for yaml changes to take effect
pot_range = rospy.get_param('/range')
#mins = rospy.get_param('/mins')

def callback(data):
    #Initialize variables:
    global pot1, pot2, pot3, pot4, pot5, pot6, pot7, pot8
    global pot1_min, pot2_min, pot3_min, pot4_min, pot5_min, pot6_min, pot7_min, pot8_min

    #"""
    #Capture initial position of potentiometers and save as minimum values:
    global count
    global mins
    while True:
        if (count==1):
            break
        else:
            mins = { 'pot1_min' : data.pot1, 'pot2_min' : data.pot2,
            'pot3_min' : data.pot3, 'pot4_min' : data.pot4,
            'pot5_min' : data.pot5, 'pot6_min' : data.pot6,
            'pot7_min' : data.pot7, 'pot8_min' : data.pot8, }
            config = client.update_configuration(mins)
            count = 1
    #"""

    #Range to normalize data to: [a, b]
    a = 0
    b = 1.57

    #To scale variable x into range [a, b]: x_scaled = (b-a)((x-min(x))/(max(x)-min(x))+a
    #Use the following to dynamically change the pot_min from gui:
    #pot1 = -b*((data.pot1 - rospy.get_param('/pot_mins_node/pot1_min'))/(pot_range))
    pot1 = -b*((data.pot1 - mins['pot1_min'])/(pot_range))
    pot2 = -b*((data.pot2 - mins['pot2_min'])/(pot_range))
    pot3 = -b*((data.pot3 - mins['pot3_min'])/(pot_range))
    pot4 = -b*((data.pot4 - mins['pot4_min'])/(pot_range))
    pot5 = -b*((data.pot5 - mins['pot5_min'])/(pot_range))
    pot6 = -b*((data.pot6 - mins['pot6_min'])/(pot_range))
    pot7 = -b*((data.pot7 - mins['pot7_min'])/(pot_range))
    pot8 = -b*((data.pot8 - mins['pot8_min'])/(pot_range))

    pot_vals = [pot1, pot2, pot3, pot4, pot5, pot6]
    for x in range (0,6):
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

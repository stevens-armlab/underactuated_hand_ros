#!/usr/bin/env python

#HOW TO USE THIS SCRIPT:
#Make sure you source before each of the following commands: 'source ~/underactuated_ws/devel/setup.bash'
#Run 'roslaunch hand_rviz display.launch' to open rviz
#Run 'rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600' to connect to Teensy
#Navigate to this file's location and run './underactuated_hand_sim.py'

import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from hand_arduino.msg import underactuatedHandSensors
#For dynamic parameters:
import dynamic_reconfigure.client

#In display.launch, must add '<rosparam param="source_list">["/joint_states_command"]</rosparam>'
#to joint_state_publisher_gui node

print(r"""
  _    _           _                     _               _           _
 | |  | |         | |                   | |             | |         | |
 | |  | |_ __   __| | ___ _ __ __ _  ___| |_ _   _  __ _| |_ ___  __| |
 | |  | | '_ \ / _` |/ _ \ '__/ _` |/ __| __| | | |/ _` | __/ _ \/ _` |
 | |__| | | | | (_| |  __/ | | (_| | (__| |_| |_| | (_| | ||  __/ (_| |
  \____/|_| |_|\__,_|\___|_|  \__,_|\___|\__|\__,_|\__,_|\__\___|\__,_|
                     | |  | |               | |
                     | |__| | __ _ _ __   __| |
                     |  __  |/ _` | '_ \ / _` |
                     | |  | | (_| | | | | (_| |
                     |_|  |_|\__,_|_| |_|\__,_|

""")

print("Please ensure hand is fully extended.")
raw_input("Press ENTER to continue...")
#Name node hand_sim
rospy.init_node('hand_sim', anonymous=True)

#For dynamic reconfigure, see server.py
client = dynamic_reconfigure.client.Client('pot_mins')

def joint_message():
    message = JointState()
    #Must match order of message.name:
    message.position = [roll, pot0, pot1, roll, pot2, pot3, pot4, pot5]
    message.name = ["finger1_roll_joint", "finger1_prox_joint", "finger1_dist_joint", "finger2_roll_joint", "finger2_prox_joint", "finger2_dist_joint", "thumb_prox_joint", "thumb_dist_joint"]

    joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)
    joint_state_pub.publish(message)

count = 0

#Assign ranges from yaml file (these are hardcoded in config/pot_ranges.yaml)
pot_ranges = rospy.get_param('/pot_ranges')

def callback(data):
    #Initialize variables:
    global pot0, pot1, pot2, pot3, pot4, pot5, roll

    #Capture initial position of potentiometers and save as minimum values:
    global count
    global mins
    pot_mins = rospy.get_param('/pot_mins')
    while True:
        if (count==1):
            break
        else:
            mins = { 'pot0' : data.pot0, 'pot1' : data.pot1,
            'pot2' : data.pot2, 'pot3' : data.pot3,
            'pot4' : data.pot4, 'pot5' : data.pot5,
            }
            config = client.update_configuration(mins)
            count = 1

    #Range to normalize data to: [a, b]
    a = 0
    b = 1.57
    roll_min = 1000 #This is the zero position
    roll_range = 900

    #To scale variable x into range [a, b]: x_scaled = (b-a)((x-min(x))/(max(x)-min(x))+a
    pot0 = b*((data.pot0 - pot_mins['pot0'])/(pot_ranges['pot0']))
    pot1 = -b*((data.pot1 - pot_mins['pot1'])/(pot_ranges['pot1']))
    pot2 = -b*((data.pot2 - pot_mins['pot2'])/(pot_ranges['pot2']))
    pot3 = b*((data.pot3 - pot_mins['pot3'])/(pot_ranges['pot3']))
    pot4 = b*((data.pot4 - pot_mins['pot4'])/(pot_ranges['pot4']))
    pot5 = -b*((data.pot5 - pot_mins['pot5'])/(pot_ranges['pot5']))
    roll = -b*((data.roll-roll_min)/(roll_range))

    pot_vals = [pot0, pot1, pot2, pot3, pot4, pot5]
    print("Sensor values:")
    for x in range (0,6):
        print("Pot " + str(x) + ":" ),
        print(pot_vals[x])
    print("Roll:"),
    print(data.roll)
    print("")

    #Execute joint_message function above
    joint_message()

def listener():
    #Node subscribes to potpub topic which is of type underactuatedHandSensors. When new messages are received,
    #callback is invoked with the message as the first argument.
    rospy.Subscriber('pot_pub', underactuatedHandSensors, callback)
    rospy.spin()

if __name__ == '__main__':

    listener()

#Prevents the buffer from being overloaded with messages
while not rospy.is_shutdown():
    	rate.sleep()

# Underactuated Robot Hand ROS Communciation
This repo currently holds the files needed to enable potentiometer communication and simulation in rviz.
## Installation Instructions
1. Make a new workspace: underactuated_ws
2. Clone this repo into the src folder of your workspace
2. Build the packages by running: `catkin_make`

## To start rviz simulation
Remember to source your catkin workspace for each unique terminal with `source ~/underactuated_ws/devel/setup.bash`
1. Download arduino code "smoothed_hand_sensors.ino" to Teensy
- Make sure you download the necessary arduino libraries:
- Download "underactuatedHandSensors.h" and place it in in your arduino ros_lib folder
- Download "Smoothed.h" and place in your Arduino libraries folder
2. Run `roslaunch hand_rviz display.launch`
- This will open rviz with the URDF of the hand
3. Run `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600` to connect rosserial to Teensy
4. Run `rosrun hand_rviz underactuated_hand_sim.py` 
After confirming the hand is fully opened, rviz should accurately simulate the joint positions of each finger.

# Underactuated Robot Hand ROS Communciation
This repo currently holds the files needed to enable potentiometer communication and simulation in rviz.
## Installation Instructions
1. Clone this repo into the src folder of your catkin workspace
2. Build the packages by running: `catkin_make`

## To start rviz simulation
Remember to source your catkin workspace for each unique terminal with `source ~/catkin_ws/devel/setup.bash`
1. Download arduino code "nasa_hand_pots.ino" to Teensy
- Make sure you download the necessary custom arduino library "nasaHandPots.h" and place it in in your arduino ros_lib folder
2. Run `roslaunch nasa_hand_urdf display.launch`
- This will open rviz with the URDF of the hand
3. Run `osrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600` to connect rosseriak to Teensy
4. Run `rosrun nasa_hand_msgs server.py` to start the server that controls the ROS parameters (for zeroing potentiometers)
5. Navigate to nasa_hand_script_param.py location and run `./nasa_hand_script_param.py`
After confirming the hand is fully opened, rviz should accurately simulate the joint positions of each finger.

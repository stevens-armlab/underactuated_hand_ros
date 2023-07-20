# Underactuated Robot Hand ROS Communciation
This repo currently contains the ros packages and arduino scripts needed to enable teleoperation of the hand via gamepad and hand (finger joint) simulation in rviz.

## Installation Instructions
1. Make a new workspace: underactuated_ws
2. Clone this repo into the src folder of your workspace
2. Build the packages by running: `catkin_make`

Build the necessary Arduino libraries:
- Run `source ~/underactuated_ws/devel/setup.bash`
- Run `rosrun rosserial_arduino make_libraries.py "location of your Arduino libraries folder"`
- This should generate a `ros_lib` folder that contains the necessary libraries to compile and upload the arduino script

*Note:* You must delete `ros_lib` if it already exists or the command will not generate the new libraries

## Quickstart: teleoperation and rviz simulation
1. Upload `underactuated_teleoperated_control_daisychain.ino` to the Teensy
2. Run `roslaunch hand_rviz teleop_sim.launch` to enable teleoperation and open rviz
3. Run `rosrun hand_rviz underactuated_hand_sim.py` to enable live simulation in rviz

## To only enable teleoperation of hand with Logitech Gamepad F710
Remember to source your catkin workspace for each unique terminal with `source ~/underactuated_ws/devel/setup.bash`

- Make sure joy package is installed by running `sudo apt-get install ros-<your_ros_version>-joy`

1. Upload `underactuated_teleoperated_control_daisychain.ino` to the Teensy
2. Plug in usb receiver for gamepad
3. Run `roslaunch hand_arduino hand.launch` to connect Teensy to rosserial and to start the `joy_node`
4. In a new terminal run `rosrun hand_arduino hand_teleop.py`

You should now be able to control the hand with the gamepad.

Controls:
- Right bumper (RB): grasps hand
- Left bumper (LB): ungrasps hand
- B button: spreads fingers
- X button: unspreads fingers

*Note:* if you get an error `IndexError: tuple index out of range`, (or the hand is not responding to the controller) that means ros is recongizing another controller as joy input. See [Configuring and Using a Linux-Supported Joystick with ROS](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) to determine the name of the controller. Once you have determined which jsX is the correct one, you can edit it in line 6 of hand.launch:
- `<param name="dev" value="/dev/input/jsX" />` (replace X with the number you have just identified)

## To only start rviz simulation
Remember to source your catkin workspace for each unique terminal with `source ~/underactuated_ws/devel/setup.bash`

1. Install joint state publisher GUI by running `sudo apt install ros-<your_ros_version>-joint-state-publisher-gui`
2. Run `roslaunch hand_rviz display.launch`
- This will open rviz with the URDF of the hand and joint_state_publisher_gui to see how each joint is being actuated
- The previous launch file for teleoperaton aleady enables rosserial connection, so it is not needed again
3. Run `rosrun hand_rviz underactuated_hand_sim.py` to start hand simulation script. After confirming the hand is fully opened, rviz should accurately simulate the joint positions of each finger. 
- This terminal will show the values of the potentiometers and the roll motor that are being sent to rviz.

## To record and play back rosbag file
1. Go to bagfiles directory with `cd ~/underactuated_ws/bagfiles/`
2. Run `rosbag record -O file_name  /pot_pub /joint_states_command /joint_states /joy /force_torque_sensor` to begin recording. All published messages will be recorded. Ctrc^C to stop.
3. Run `rosbag play file_name` to play back the recording. 
Note: must stop `hand_teleop.py` and `underactuated_hand_sim.py` before playback or movements will be janky.

*Tips*:
-  Make sure you have all the other necessary Arduino libraries installed.
- Make sure the correct Teensy port is defined in the launch file (can be confirmed from Arduino IDE)


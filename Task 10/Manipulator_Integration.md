# Installation
Install the manipulator packages through the following lines of code:
```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

# Installation Checks
After running the installation script on the robot computer, verify that it was successful in finding the U2D2 by checking that the port name shows up as ttyDXL. The command and the expected output are below:
```
ls /dev | grep ttyDXL
```
which should give the output:
```
-ttyDXL
```

# Testing
This guide is intended to get the use familiar with the basic functions and interfaces of the ROS 2 Interface.
Get familiar with the virtual robot model by launching it in RViz and playing with the joint_state_publisher. Note that you must specify which arm model is being used as a command line argument. For example, the PX-150 robot arm can be launched with the command below. Make sure to press Ctrl + C in the terminal when to end the session you’re done.
```
ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=px150 use_joint_pub_gui:=true
```
Get familiar with the physical robot arm (we’ll use a ViperX-250 as an example) by executing the following command in the terminal:
```
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150
```
By default, all the motors in the robot are torqued on so it will be very difficult to manually manipulate it. To torque off all the motors, execute the command below in another terminal.
```
ros2 service call /vx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
```
Now you should be able to freely manipulate the arm and gripper. Take note of how the RViz model accurately mimics the real robot. To make the robot hold a certain pose, manually hold the robot in the desired pose and execute the following command:
```
ros2 service call /vx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```
Let go and observe how the arm stays in place. Hold on to the robot and shutdown all nodes by pressing Ctrl + C in the terminal where you started the launch file.
Another way to check if all functions work is to launch the interbotix_xsarm_joy package. This package allows you to control your arm using a Bluetooth controller. See it’s usage details in its documentation page.

# Joystick Control

![image](https://github.com/Team-7-UOM/DocumentationForLeoRover/assets/66565433/3d2668c6-baf3-482a-8975-9173e827d71f)
As shown above, the interbotix_xsarm_joy package builds on top of the interbotix_xsarm_control package. To get pointers about the nodes in that package, please look at its README. The other nodes are described below:

joy - a ROS driver for a generic Linux joystick; it reads data from a joystick over Bluetooth and publishes sensor_msgs/msg/Joy messages to the commands/joy_raw topic
xsarm_joy - responsible for reading in raw sensor_msgs/msg/Joy messages from the commands/joy_raw topic and converting them into ArmJoy messages; this makes the code more readable and allows users to remap buttons very easily later.
xsarm_robot - responsible for reading in ArmJoy messages and sending joint and gripper commands to the xs_sdk node; while the ‘waist’ joint can be directly controlled via the PS3/PS4 joystick, other buttons allow position-ik to be performed using all the arm joints.

Use the Joystick pairing used in Week 9 to pair the controller. Go to the terminal and type the following code after pairing the controller:
```
ros2 launch interbotix_xsarm_joy xsarm_joy.launch.py robot_model:=px150
```

To understand how the joystick buttons map to controlling the robot, look at the diagram and table below.

![image](https://github.com/Team-7-UOM/DocumentationForLeoRover/assets/66565433/bf579971-74d5-4f4e-ba48-4a91f8718978)

| Button             | Action                                                   |
|---------------------|----------------------------------------------------------|
| START/OPTIONS      | Move robot arm to its Home pose                           |
| SELECT/SHARE       | Move robot arm to its Sleep pose                          |
| R2                  | Rotate the ‘waist’ joint clockwise                        |
| L2                  | Rotate the ‘waist’ joint counterclockwise                |
| Triangle            | Increase gripper pressure in 0.125 step increments (max is 1) |
| X                   | Decrease gripper pressure in 0.125 step increments (min is 0) |
| O                   | Open gripper                                             |
| Square              | Close gripper                                            |
| D-pad Up            | Increase the control loop rate in 1 Hz step increments (max of 40) |
| D-pad Down          | Decrease the control loop rate in 1 Hz step increments (min of 10) |
| D-pad Left          | Coarse control - sets the control loop rate to a user-preset ‘fast’ rate |
| D-pad Right         | Fine control - sets the control loop rate to a user-preset ‘slow’ rate |
| Right stick Up/Down | Increase/Decrease pitch of the end-effector               |
| Right stick Left/Right | Increase/Decrease roll of the end-effector              |
| R3                  | Reverses the Right stick Left/Right control              |
| Left stick Up/Down  | Move the end-effector (defined at ‘ee_gripper_link’) vertically in Cartesian space |
| Left stick Left/Right | Move the end-effector (defined at ‘ee_gripper_link’) horizontally in Cartesian space |
| L3                  | Reverses the Left stick Left/Right control               |
| R1                  | If the arm has 6dof, this moves the end-effector in a negative direction along its own ‘y’ axis |
| L1                  | If the arm has 6dof, this moves the end-effector in a positive direction along its own ‘y’ axis |
| PS                  | If torqued on, holding for 3 seconds will torque off the robot; if torqued off, tapping the button will torque on the robot |

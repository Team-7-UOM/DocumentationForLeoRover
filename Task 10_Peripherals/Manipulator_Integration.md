# Direct Connection to the NUC
## Installation
Install the manipulator packages through the following lines of code:
```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

## Installation Checks
After running the installation script on the robot computer, verify that it was successful in finding the U2D2 by checking that the port name shows up as ttyDXL. The command and the expected output are below:
```
ls /dev | grep ttyDXL
```
which should give the output:
```
-ttyDXL
```

## Testing
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

## Joystick Control

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

# Connection to the Rover
## Wiring and electronics connection
Stick the Dynamixel cable coming out of the base of the arm into the power distribution board.
Connect U2D2 and the power distribution board with the short Dynamixel cable.
Connect the U2D2 to the rover using an USB cable
The last step is to stick connect the barrel jack cable to the battery power supply ( a powerbox might be useful here) and plug into the other end into power distribution board.
With everything connected it should look like this:

![image](https://github.com/Team-7-UOM/DocumentationForLeoRover/assets/66565433/a90d6803-b94b-4cdc-add5-ba0720f0fb66)

## Package installation
Open text editor and paste the following lines of code:
```
#!/bin/bash -e

get_ws_path() {
    # Ask the user for the ROS workspace path
    read -p "Input absolute path to the ROS 2 workspace, you want to install the packages into:" ws_path
    echo "Your workspace: $ws_path"
}

install_essentials() {
    echo -e "Installing pip3.\n"
    sudo apt -y install python3-pip

    echo -e "Installing modern_robotics package"
    pip3 install modern_robotics
}

install_interbotix_packages() {
    source $ws_path/install/setup.bash || true
    cd $ws_path/src
    echo -e "Cloning repositories. \n"
    git clone https://github.com/Interbotix/interbotix_ros_core.git -b humble
    git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b humble
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b humble
    
    echo -e "Removing CATKIN_IGNORE files. \n"
    rm interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
    rm interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE
    rm interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
    rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE

    echo -e "Adding UDEV rules for robotic ARM. \n"
    cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger

    echo -e "Installing ROS dependencies. \n"
    cd $ws_path
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

    echo -e "Building and sourcing the workspace. \n"
    colcon build --symlink-install --packages-skip interbotix_ros_*
    source install/setup.bash
}

################################## MAIN
get_ws_path

# Ask user for confirmation of the path
read -p "Is the path correct? [Y/n]" -n 1 -r 

REPLY=${REPLY,,} #lowercase answer

if [[ "$REPLY" =~ ^(no|n)$ ]]
then
    echo
    get_ws_path
else
    echo -e "Processing installation."
fi

#################### 1) installing essential packages
install_essentials

#################### 2) installing ROS packages
install_interbotix_packages

echo -e "Successfully installed Interbotix ROS packages and required dependencies."
```
and save it as **install_interbotix_packages.sh**
and right click on the file and run it on the terminal.
You will be asked to provide the absolute path to your ROS2 workspace (if you don't have one on the rover, check this tutorial first). Type it in and confirm with enter.
Once the packages have been built, you can edit the environmental setup file to point to your result space. Open the file in nano:
```
nano /etc/ros/setup.bash
```
Comment out the first line by adding the # sign and add the source command for your workspace. The first 2 lines should look essentially like this:
```
# source /opt/ros/melodic/setup.bash
source /home/pi/ros2_ws/devel/setup.bash
```
Different operations on the arm require different modes for the ros_controller. Instead of changing the package configs, we will make our own config file, and load it in the launch file. First, make your config file and open it to edit it:
```
touch /etc/ros/arm_modes.yaml
nano /etc/ros/arm_modes.yaml
```
Then, paste these lines in the file (ctrl+shift+v):
```
groups:
  arm:
    operating_mode: position
    profile_type: time
    profile_velocity: 131
    profile_acceleration: 25
    torque_enable: true

singles:
  gripper:
    operating_mode: pwm
    profile_type: velocity
    profile_velocity: 131
    profile_acceleration: 15
    torque_enable: true
```
To save and exit the file, you can use respectively ctrl+o and ctrl+x.

Now, to add the arm's driver to the rover's launch file, open the robot.launch.py file:
```
nano /etc/ros/robot.launch.py
```
and paste these lines somewhere between the existing nodes:
```
Node(
            package=xsarm_ros_control_package,
            executable=xsarm_ros_control_launch_file,
            output='screen',
            arguments=[
                f'robot_model:=px150',
                f'dof:=4',
                f'use_world_frame:=false',
                f'mode_configs:=/etc/ros/arm_modes.yaml'
            ]
        )
```
You can also edit the robot's URDF file to connect the arm's base link to the rover's model. To do this, open the robot.urdf.xacro file:
```
nano /etc/ros/urdf/robot.urdf.xacro
```
and paste these lines somewhere between the <robot> tags:
```
<link name="px150/base_link"/>

<joint name="arm_joint" type="fixed">
  <origin xyz="0.043 0 -0.001"/>
  <parent link="base_link"/>
  <child link="px150/base_link"/>
</joint>
```
That's it! On the next boot, the arm driver node will start together with all the other nodes. You can manually restart the running nodes by typing:
```
sudo systemctl restart leo
```

# wArm_ros
This repo contains all the necessary files to move wArm robot arm using ROS.


## Instructions

1. Make sure you have ROS installed correctly with a functioning workspace-- I used ROS Kinetic on Ubuntu 16.04. I currently have 'wArm_ros' in the 'src' folder of my catkin workspace.

2. To execute GCODE path in simulation using Rviz, execute the following terminal command:

    - To start joint_publisher node and Rviz with the robot URDF already loaded
      ```
      roslaunch warm_description display_marker.launch
      ```
    - To start the gcode interpreter
    
      ```
      rosrun warm_gcode gcode_interpreter.py
      ```
      - Select GCODE file to execute from the window
      - Hit 'enter' to interpret it
      - If there were no errors, hit 'enter' again to bring down the pen
      - Hit 'enter' to start execution

### Moving the real robot, synced with the simulated robot's trajectories.

3. Make sure you download the AccelStepper and ros_lib libraries into your Arduino environment.
	- If ros_lib already exists in your Arduino libraries (<Arduino sketchbook>/libraries), follow the last troubleshooting tip or you'll get an error saying "ArmJointState.h: no such file".  ROS makes you remove ros_lib and regenerate it every time you introduce a new custom message.

4. Make sure the robot and the simulation are in the same position (real robot must be positioned as in Rviz simulation)

5. With the simulation already running, execute each of the following commands in it's own, separate terminal: 
	- ```rosrun rosserial_python serial_node.py /dev/ttyACM0``` (establishes rosserial node that communicates with Arduino on ACM0)
	- ```rosrun warm_arduino JointAngle2Steps``` (converts simulation joint_state rotations to steps and publishes on the /joint_steps topic, which the Arduino script subscribes to)

**Now, whatever trajectories are planned and executed in simulation are echoed on the real robot.**

## Included directories
### warm_arduino
- *warm_arduino.ino*: Arduino sketch
- *JointAngle2Steps.cpp*: node to convert angles from */joint_states* topic from radiants to motor steps. Results are published on */joint_steps* topic

### warm_description
- URDF (Unified Robot Description File) for wArm (including 3D meshes) necessary for simulation in Rviz 
- *joint_publisher.cpp*
  - Receives *(x,y,z)* coordinates on */coordinates* topic
  - Compute joint angles *q1,q2,q3* via inverse kinematics function
  - Publish joint angles to */joint_states* topic

### warm_gcode
- *gcode_interpreter.py*: gcode interpreter node
- some GCODE examples


## Other features

- **To move wArm to a desired position using command line:**
	- ```rostopic pub /coordinates geometry_msgs/Point "x: x_coord y: y_coord z: z_coord"```  
	- Change "x_coord, y_coord, z_coord" with the desired coordinates

## Troubleshooting

- If you get the following ```"error: warm_arduino/ArmJointState.h: No such file or directory"```, perform the following steps in terminal:
	```
	cd <Arduino sketchbook>/libraries
	rm -rf ros_lib 
	rosrun rosserial_arduino make_libraries.py .
	```

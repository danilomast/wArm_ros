#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "warm_arduino/ArmJointState.h"
#include "math.h"

warm_arduino::ArmJointState joints;
warm_arduino::ArmJointState arm_steps;
warm_arduino::ArmJointState total;
int joint_status = 0;
int count = 0;
double prev_angle[3] = {0,0,0}; 

//robot parameters
int stepsPerRevolution=6400;
float gearRatio = 5.3;



//Convertion from angle in radiants to motor steps
void chatterCallback(const sensor_msgs::JointState& cmd_arm)
{

  ROS_INFO("I received: [%f][%f][%f]", cmd_arm.position[0], cmd_arm.position[1],cmd_arm.position[2]);
  
    if (count==0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
  }
  arm_steps.position1 = (int)((cmd_arm.position[0]-prev_angle[0])*gearRatio*stepsPerRevolution/(2*M_PI));
  arm_steps.position2 = (int)((cmd_arm.position[1]-prev_angle[1])*gearRatio*stepsPerRevolution/(2*M_PI));
  arm_steps.position3 = (int)((cmd_arm.position[2]-prev_angle[2])*gearRatio*stepsPerRevolution/(2*M_PI));


  if (count!=0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];

  }

  total.position1 += arm_steps.position1;
  total.position2 += arm_steps.position2;
  total.position3 += arm_steps.position3;


  ROS_INFO("Motor steps: [%d][%d][%d]", total.position1, total.position2,total.position3);

  joint_status=1;
  count=1;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "JointAngle2Steps");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 1000, chatterCallback);
  ros::Publisher pub = n.advertise<warm_arduino::ArmJointState>("joint_steps",50);
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    if(joint_status==1)
    {
      joint_status = 0;
      pub.publish(total);
    }
    ros::spinOnce();
    loop_rate.sleep();  
  }

}
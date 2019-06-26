#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <iostream>
#include <visualization_msgs/Marker.h>

#define MATH_PI 				3.141592653589793238463
#define MATH_TRANS  			57.2958 

// Lenght of forearm and arm link
#define MATH_L1 				200 //mm
#define MATH_L2 				214 //mm

// Limits of Joint 2 and 3
#define J2_MIN					-1  //rad
#define J2_MAX 					0.8 //rad  

#define J3_MIN					0 //rad
#define J3_MAX					1.3 //rad  


float angle[3]={0.0};
float position[3]={0.0};



//Check if IK result angles are reachable by the robot 

bool reachable(float q2, float q3)
{
	if((q2 < J2_MAX) && (q2> J2_MIN))
		if((q3 < J3_MAX) && (q3> J3_MIN))
			return true;
	return false;
}


//Inverse kinematic function: given desired position x,y,z returns joint angles q1,q2,q3

bool warm_ik(float position[3], float angle[3])
{
	float q1, q2, q3 = 0.0;

	float axis_scaling[3]={1,1,1};  
    float offset[3]={0,0,-70};

	int X_AXIS=0;
	int Y_AXIS=1;
	int Z_AXIS=2;

	float rob_pos[3]; 
	float CencerOffset=30;
	float HeadOffset= 66.8; //20;

	static float C2, S2, K1, K2, theta, psi;
	static float ARM_XYZ, ARM_XY; 
	float TempDelta; 

	rob_pos[X_AXIS] = position[X_AXIS] * axis_scaling[X_AXIS] + offset[X_AXIS]; 
	rob_pos[Y_AXIS] = position[Y_AXIS] * axis_scaling[Y_AXIS] + offset[Y_AXIS]; 
	rob_pos[Z_AXIS] = position[Z_AXIS] * axis_scaling[Z_AXIS] + offset[Z_AXIS]; 

	ARM_XY = sqrt(pow(rob_pos[X_AXIS],2) + pow(rob_pos[Y_AXIS],2)) - CencerOffset - HeadOffset; 
	ARM_XYZ = sqrt(pow(ARM_XY,2) + pow(rob_pos[Z_AXIS],2)); 
	
	C2 = (pow(ARM_XYZ,2) - pow(MATH_L1,2) - pow(MATH_L2,2))/(2 * MATH_L1 * MATH_L2); 
	S2 = sqrt(1 - pow(C2,2) ); 
	K1 = MATH_L1 + MATH_L2 * C2; 
	K2 = MATH_L2 * S2; 
	theta = (atan2(rob_pos[Z_AXIS],ARM_XY) + atan2(K2, K1)); 
	psi = atan2(S2, C2); 
	
	
	q1 = atan2(rob_pos[Y_AXIS],rob_pos[X_AXIS]);
	q2 = MATH_PI/2 - theta;
	q3 = psi - theta;


	if (isnan(q1) || isnan(q2) || isnan(q3))
		return false;

	printf("q1: %f\n",q1);
	printf("q2: %f\n",q2);
	printf("q3: %f\n",q3);

	if (reachable(q2,q3)){
		angle[0] = q1;
		angle[1] = q2;
		angle[2] = q3;
		return true;
	}
		return false;

}



void coord_Callback(const geometry_msgs::Point& msg)
{

	printf("Coordinate received!\n");
	
	position[0] = msg.x;
	position[1] = msg.y;
	position[2] = msg.z;
	
	if (!warm_ik(position, angle))
		ROS_ERROR("Inverse kinematic is wrong");	
}


void reset_Callback(const std_msgs::Bool& msg)
{
	angle[0]=0;
	angle[1]=0;
	angle[2]=0;
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle n;

    // Publishers and subscribers
   	ros::Subscriber sub = n.subscribe("coordinates", 1,coord_Callback );
   	ros::Subscriber sub_reset = n.subscribe("reset_angles", 1,reset_Callback );
    ros::Publisher joint_pub  = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double  joint_1=0, joint_2=0, joint_3=0;
    sensor_msgs::JointState joint_state;
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    while (ros::ok()) {

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        
       	joint_state.name[0] ="joint_1";
        joint_state.position[0] = angle[0];
        joint_state.name[1] ="joint_2";
        joint_state.position[1] = angle[1];
       	joint_state.name[2] ="joint_3";
       	joint_state.position[2] = angle[2];

        //send the joint state and transform
        joint_pub.publish(joint_state);


        //Creates a spherial marker on end-effector desired position

		visualization_msgs::Marker marker;

   	 	marker.header.frame_id = "/base_link";
   		marker.header.stamp = ros::Time::now();
    	marker.ns = "basic_shapes";
    	marker.id = 0;
    	marker.type = shape;
    	marker.action = visualization_msgs::Marker::ADD;


    	marker.pose.position.x = position[0]/1000;
    	marker.pose.position.y = position[1]/1000;
    	marker.pose.position.z = position[2]/1000;

    	marker.pose.orientation.x = 0.0;
    	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
    	marker.pose.orientation.w = 1.0;

    	marker.scale.x = 0.02;
    	marker.scale.y = 0.02;
    	marker.scale.z = 0.02;
 
    	marker.color.r = 0.0f;
    	marker.color.g = 1.0f;
    	marker.color.b = 0.0f;
    	marker.color.a = 1.0;

    	marker.lifetime = ros::Duration();

    	marker_pub.publish(marker);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
/* This sketch uses ROS as well as AccelStepper libraries to control the 
 * wArm robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to joint_steps:
 *       joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.      
 * Publishing to calib (not yet used in the current version):      
 *       Tells ROS that calibration of robot is done     
 */

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>
#include <warm_arduino/ArmJointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


//Pin definition (RAMPS 1.4)

// Joint 1
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

// Joint 2
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

// Joint 3
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19


AccelStepper joint1(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint2(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint3(1,Z_STEP_PIN, Z_DIR_PIN);
MultiStepper steppers;


//global variables
int joint_step[3];
bool calib=0;
int joint_status = 0;
int calib_joint1=0;
int calib_joint2=0;
int calib_joint3=0;
ros::NodeHandle nh;
std_msgs::Int16 msg;
std_msgs::Bool calib_done;


void arm_cb(const warm_arduino::ArmJointState& arm_steps)
{
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  calib=arm_steps.calib;
}


//instantiate publisher for calibration
ros::Publisher calib_pub("calib_feedback",&calib_done);

//instantiate subscriber
ros::Subscriber<warm_arduino::ArmJointState> arm_sub("joint_steps",arm_cb); //subscribes to joint_steps on arm

void setup() {

  joint1.setEnablePin(X_ENABLE_PIN); 
  joint1.setPinsInverted(false, false, true); //invert logic of enable pin
  joint1.enableOutputs();
  
  joint2.setEnablePin(Y_ENABLE_PIN); 
  joint2.setPinsInverted(false, false, true); //invert logic of enable pin
  joint2.enableOutputs();
  
  joint3.setEnablePin(Z_ENABLE_PIN); 
  joint3.setPinsInverted(false, false, true); //invert logic of enable pin
  joint3.enableOutputs();

  pinMode(13,OUTPUT);

  //Endstops pins
  pinMode(X_MIN_PIN, INPUT);     
  pinMode(Y_MIN_PIN, INPUT); 
  pinMode(Z_MIN_PIN, INPUT); 
  
  joint_status = 1;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.advertise(calib_pub);

  // Configure each stepper
  joint1.setMaxSpeed(500);
  joint1.setSpeed(500); 
  
  joint2.setMaxSpeed(500);
  joint2.setSpeed(500); 

  joint3.setMaxSpeed(-500);
  joint3.setSpeed(-500); 
 
  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);

  digitalWrite(13, 1); //toggle led
}

void loop() {
  
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[3];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; 
    positions[1] = -joint_step[1];  // negated since the real robot rotates in the opposite direction as ROS
    positions[2] = joint_step[2]; 


    //If calibration command is received from ROS, then start the procedure
    if(calib)
    {
        while((calib_joint1 && calib_joint2 && calib_joint3)==false)
        
        {
          //Move all the motors until all endstops are reached
          if(digitalRead(X_MIN_PIN)==HIGH)
            joint1.runSpeed();    
          else
            calib_joint1=true;
          
          if(digitalRead(Y_MIN_PIN)==HIGH)
            joint2.runSpeed();      
          else
            calib_joint2=true;
              
          if(digitalRead(Z_MIN_PIN)==HIGH)
            joint3.runSpeed();    
          else
            calib_joint3=true;   
        }

       //Reset stepper positions
      joint1.setCurrentPosition(0);
      joint2.setCurrentPosition(0);
      joint3.setCurrentPosition(0);

      //Set-up normal stepper velocity
      joint1.setMaxSpeed(500);
      joint2.setMaxSpeed(500);
      joint3.setMaxSpeed(500);

      joint1.setSpeed(500);
      joint2.setSpeed(500);
      joint3.setSpeed(500);

      //tell ROS the end of calibration
      calib_done.data=1;
      calib_pub.publish(&calib_done);
      nh.spinOnce();
    }
    
    else
    {  
      //move the steppers
      steppers.moveTo(positions);
      nh.spinOnce();
      steppers.runSpeedToPosition(); // Blocks until all are in position 
    }
  }
  
  joint_status = 0;
  nh.spinOnce();
  delay(1);
  
}

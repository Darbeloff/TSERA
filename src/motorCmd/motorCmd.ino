#include "motorClass.h"
#include "calibration.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

//instantiate motors!

//stage 1
motorClass m1a =  motorClass(8,33,44,49);
motorClass m1b =  motorClass(9,32,48,45);
motorClass m1c =  motorClass(11,30,46,47);

//stage 2
motorClass m2a =  motorClass(7,34,38,41);
motorClass m2b =  motorClass(6,35,42,39);
motorClass m2c =  motorClass(4,37,40,43);

//stage 3
motorClass m3a =  motorClass(2,29,22,25); //1-1
motorClass m3b =  motorClass(3,28,24,23); //1-2
motorClass m3c =  motorClass(10,31,26,27); //2-1



int vel_or_pos = 1; //position control
int calibrated = 0;//initially not calibrated

void command_callback(const std_msgs::Float32MultiArray& setpoint){

  //if (setpoint.data[9]==1){
    m1a.setMotorPos(setpoint.data[0]);
    m1b.setMotorPos(setpoint.data[1]);
    m1c.setMotorPos(setpoint.data[2]);
  
    m2a.setMotorPos(setpoint.data[3]);
    m2b.setMotorPos(setpoint.data[4]);
    m2c.setMotorPos(setpoint.data[5]);
  
    
    m3a.setMotorPos(setpoint.data[6]);
    m3b.setMotorPos(setpoint.data[7]);
    m3c.setMotorPos(setpoint.data[8]);
  //}
  //else{
//    vel_or_pos = 0; //velocity control-- for the purpose of calibration
//    calibrated = 0; //not calibrated
//    }
  
}


////ros
ros::NodeHandle ArduinoInterface;
std_msgs::Float32MultiArray outputVelocity;
std_msgs::Float32 squareError_msg;

//create publisher

ros::Publisher squareError("squareError", &squareError_msg);

//subscriber
ros::Subscriber<std_msgs::Float32MultiArray> velSub("ik", &command_callback);



void setup () {

  delay(1000);

  //initialize ros
  ArduinoInterface.initNode();


  //publish
  ArduinoInterface.advertise(squareError);
  

  //subscribe
  ArduinoInterface.subscribe(velSub);
  delay(1000);

}


void loop (){

//
//  if (calibrated==1){
    m3a.pos_closedLoopController();
    m3b.pos_closedLoopController();
    m3c.pos_closedLoopController();
  
    m2a.pos_closedLoopController();
    m2b.pos_closedLoopController();
    m2c.pos_closedLoopController();
    
    m1a.pos_closedLoopController();
    m1b.pos_closedLoopController();
    m1c.pos_closedLoopController();
//
//  }
//  else{
//    calibrate(m3a,m3b,m3c);
//    calibrate(m2a,m2b,m3c);
//    calibrate(m1a,m1b,m1c);
//    }

  squareError_msg.data = pow(m1a.errorPos,2)+ pow(m1b.errorPos,2)+ pow(m1c.errorPos,2)+pow(m2a.errorPos,2)+ pow(m2b.errorPos,2)+ pow(m2c.errorPos,2)+pow(m3a.errorPos,2)+ pow(m3b.errorPos,2)+ pow(m3c.errorPos,2);
  squareError.publish( &squareError_msg );
  
  ArduinoInterface.spinOnce();
  }









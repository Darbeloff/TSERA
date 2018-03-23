#include "motorClass.h"
#include "calibration.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

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

void command_callback(const std_msgs::Float32MultiArray& posSetpoint){
  m1a.setMotorPos(posSetpoint.data[0]);
  m1b.setMotorPos(posSetpoint.data[1]);
  m1c.setMotorPos(posSetpoint.data[2]);

  m2a.setMotorPos(posSetpoint.data[3]);
  m2b.setMotorPos(posSetpoint.data[4]);
  m2c.setMotorPos(posSetpoint.data[5]);

  
  m3a.setMotorPos(posSetpoint.data[6]);
  m3b.setMotorPos(posSetpoint.data[7]);
  m3c.setMotorPos(posSetpoint.data[8]);
}


////ros
ros::NodeHandle ArduinoInterface;
std_msgs::Float32MultiArray outputVelocity;

ros::Subscriber<std_msgs::Float32MultiArray> velSub("ik", &command_callback);



void setup () {

  delay(1000);
  //Serial.begin(9600);

  //initialize ros
  ArduinoInterface.initNode();

  //subscribe
  ArduinoInterface.subscribe(velSub);

}


void loop (){

  m3a.pos_closedLoopController();
  m3b.pos_closedLoopController();
  m3c.pos_closedLoopController();

  m2a.pos_closedLoopController();
  m2b.pos_closedLoopController();
  m2c.pos_closedLoopController();
  

  m1a.pos_closedLoopController();
  m1b.pos_closedLoopController();
  m1c.pos_closedLoopController();



  //m1b.logValues();

  
  ArduinoInterface.spinOnce();
  
  }









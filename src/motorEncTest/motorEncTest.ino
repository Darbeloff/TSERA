#include "motorClass.h"
#include "calibration.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

//instantiate motors!
//Portescap Motors
float gearRatio1 = 166.0;
float EncCntsRev1 = 16.0*4;


//Robot Shop Motors
float gearRatio2 = 103.814;
float EncCntsRev2 = 12;

//stage 1
motorClass m1a =  motorClass(8,33,44,49,gearRatio2, EncCntsRev2);
motorClass m1b =  motorClass(9,32,48,45,gearRatio2, EncCntsRev2);
motorClass m1c =  motorClass(11,30,46,47,gearRatio2, EncCntsRev2);

//stage 2
motorClass m2a =  motorClass(7,34,38,41,gearRatio1, EncCntsRev1);
motorClass m2b =  motorClass(6,35,42,39,gearRatio1, EncCntsRev1);
motorClass m2c =  motorClass(4,37,40,43,gearRatio1, EncCntsRev1);

//stage 3
motorClass m3a =  motorClass(2,29,22,25,gearRatio1, EncCntsRev1); //1-1
motorClass m3b =  motorClass(3,28,24,23,gearRatio1, EncCntsRev1); //1-2
motorClass m3c =  motorClass(10,31,26,27,gearRatio1, EncCntsRev1); //2-1



void setup () {
  Serial.begin(9600);
  m1a.setMotorPos(1);
  m1b.setMotorPos(1);
  m1c.setMotorPos(1);
}


void loop (){
  m1a.pos_closedLoopController();
  m1b.pos_closedLoopController();
  m1c.pos_closedLoopController();
  }









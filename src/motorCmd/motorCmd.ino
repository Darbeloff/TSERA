#include "motorClass.h"
#include "calibration.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//instantiate motors!
//Portescap Motors
float gearRatio1 = 166.0;
float EncCntsRev1 = 16.0*4;

//Robot Shop Motors
float gearRatio2 = 103.814;
float EncCntsRev2 = 12.0;

//stage 1
motorClass m1a =  motorClass(8,33,44,49,gearRatio2, EncCntsRev2);
motorClass m1b =  motorClass(9,32,48,45,gearRatio2, EncCntsRev2);
motorClass m1c =  motorClass(11,30,46,47,gearRatio2, EncCntsRev2);

//stage 2
motorClass m2a =  motorClass(7,34,38,41,gearRatio2, EncCntsRev2);
motorClass m2b =  motorClass(6,35,42,39,gearRatio2, EncCntsRev2);
motorClass m2c =  motorClass(4,37,40,43,gearRatio2, EncCntsRev2);

//stage 3
motorClass m3a =  motorClass(2,29,22,25,gearRatio2, EncCntsRev2); //1-1
motorClass m3b =  motorClass(3,28,24,23,gearRatio2, EncCntsRev2); //1-2
motorClass m3c =  motorClass(10,31,26,27,gearRatio2, EncCntsRev2); //2-1



int vel_or_pos = 1; //position control
int calibrated = 0;//initially not calibrated
float epsilon = 0.07;


void command_callback(const std_msgs::Float32MultiArray &setpoint){

    m1a.setMotorPos(setpoint.data[0]); //width
    m1b.setMotorPos(setpoint.data[1]);
    m1c.setMotorPos(setpoint.data[2]);
  
    m2a.setMotorPos(setpoint.data[3]);
    m2b.setMotorPos(setpoint.data[4]);
    m2c.setMotorPos(setpoint.data[5]);
  
    m3a.setMotorPos(setpoint.data[6]);
    m3b.setMotorPos(setpoint.data[7]);
    m3c.setMotorPos(setpoint.data[8]);
}


//ros
bool ROS_switch = true;
bool Error_switch = true;
ros::NodeHandle ArduinoInterface;
std_msgs::Float32MultiArray outputVelocity;
std_msgs::Float32MultiArray squareError_msg;
std_msgs::Bool ready_next;

//create publisher
ros::Publisher error_check("continueWaypoint", &ready_next);
ros::Publisher squareErrorPub("squareError", &squareError_msg);

//subscriber
ros::Subscriber<std_msgs::Float32MultiArray> velSub("ik", &command_callback);
float array_storage[9];

void setup () {

  if (ROS_switch){
    //initialize ros
    ArduinoInterface.initNode();
    squareError_msg.data_length = 9;
    squareError_msg.data = array_storage;

    //publisher
    ArduinoInterface.advertise(squareErrorPub);
    ArduinoInterface.advertise(error_check);

    
    //subscriber
    ArduinoInterface.subscribe(velSub);
    delay(1000);
  }
  else{Serial.begin(9600);
  m3c.setMotorPos(0);
  }

}


void loop (){

      m3a.pos_on_off_controller();
      m3b.pos_on_off_controller();
      m3c.pos_on_off_controller();

      m2a.pos_on_off_controller();
      m2b.pos_on_off_controller();
      m2c.pos_on_off_controller();

      m1a.pos_on_off_controller();
      m1b.pos_on_off_controller();
      m1c.pos_on_off_controller();
      
//    m3a.pos_closedLoopController();
//    m3b.pos_closedLoopController();
//    m3c.pos_closedLoopController();
  
//    m2a.pos_closedLoopController();
//    m2b.pos_closedLoopController();
//    m2c.pos_closedLoopController();
    
//    m1a.pos_closedLoopController();
//    m1b.pos_closedLoopController();
//    m1c.pos_closedLoopController();

      if(ROS_switch && Error_switch){ //Error Switch is set to stage 
        if ((sqrt(pow(m3a.errorPos,2)+ pow(m3b.errorPos,2)+ pow(m3c.errorPos,2)))< epsilon){
            ready_next.data = true;
            error_check.publish( &ready_next );
          }
          
      squareError_msg.data[0] = m3a.errorPos;
      squareError_msg.data[1] = m3b.errorPos; //first test, read encoder for all three, c was not responsive and A and B kept flickering back and forth.
      squareError_msg.data[2] = m3c.errorPos; 
      squareError_msg.data[3] = m3a.errorPos;
      squareError_msg.data[4] = m3b.errorPos; //first test, read encoder for all three, c was not responsive and A and B kept flickering back and forth.
      squareError_msg.data[5] = m3c.errorPos; 
      squareError_msg.data[6] = m3a.errorPos;
      squareError_msg.data[7] = m3b.errorPos; //first test, read encoder for all three, c was not responsive and A and B kept flickering back and forth.
      squareError_msg.data[8] = m3c.errorPos; 
      squareErrorPub.publish( &squareError_msg );
      ArduinoInterface.spinOnce();
      delay(1000);
        }
    else{
    m3c.log_on_off();
    }
  }


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
motorClass m2a =  motorClass(7,34,38,41,gearRatio1, EncCntsRev1);
motorClass m2b =  motorClass(6,35,42,39,gearRatio1, EncCntsRev1);
motorClass m2c =  motorClass(4,37,40,43,gearRatio1, EncCntsRev1);

//stage 3
motorClass m3a =  motorClass(2,29,22,25,gearRatio1, EncCntsRev2); //1-1
motorClass m3b =  motorClass(3,28,24,23,gearRatio2, EncCntsRev2); //1-2
motorClass m3c =  motorClass(10,31,26,27,gearRatio2, EncCntsRev2); //2-1



int vel_or_pos = 1; //position control
int calibrated = 0;//initially not calibrated
float epsilon;
//float squareError2[3];
//float squareError3[3];

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
std_msgs::Float32MultiArray squareError_msg1;
std_msgs::Bool ready_next;
//std_msgs::Float32 squareError_msg2;
//std_msgs::Float32 squareError_msg3;

//create publisher
ros::Publisher error_check("continueWaypoint", &ready_next);
ros::Publisher squareErrorPub1("squareError1", &squareError_msg1);
//ros::Publisher squareErrorPub2("squareError2", &squareError_msg2);
//ros::Publisher squareErrorPub3("squareError3", &squareError_msg3);


//subscriber
ros::Subscriber<std_msgs::Float32MultiArray> velSub("ik", &command_callback);
float array_storage[3];

void setup () {

  if (ROS_switch){
    //initialize ros
    ArduinoInterface.initNode();
    squareError_msg1.data_length = 3;
    squareError_msg1.data = array_storage;

    //publisher
    ArduinoInterface.advertise(squareErrorPub1);
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

//      squareError1[0] = sqrt(pow(m3a.errorPos,2)+ pow(m3b.errorPos,2)+ pow(m3c.errorPos,2));//Gives displacement error of the stage
//      squareError1[1] = m3a.errorPos;//Gives magnitude and direction of error in motor
//      squareError1[2] = m3b.errorPos;//Gives magnitude and direction of error in motor
//      squareError1[3] = m3c.errorPos;//Gives magnitude and direction of error in motor


//      squareError2[0] = pow(m2a.errorPos,2)+ pow(m2b.errorPos,2)+ pow(m2c.errorPos,2);
//      squareError2[1] = pow(m2a.errorPos,2);
//      squareError2[2] = pow(m2b.errorPos,2);
//      squareError2[3] = pow(m2c.errorPos,2);
      
//      squareError3[0] = pow(m3a.errorPos,2)+ pow(m3b.errorPos,2)+ pow(m3c.errorPos,2);
//      squareError3[1] = pow(m3a.errorPos,2);
//      squareError3[2] = pow(m3b.errorPos,2);
//      squareError3[3] = pow(m3c.errorPos,2);
    
        if(ROS_switch && Error_switch){ //Error Switch is set to stage 
          if ((sqrt(pow(m3a.errorPos,2)+ pow(m3b.errorPos,2)+ pow(m3c.errorPos,2)))< epsilon){
            ready_next.data = true;
            error_check.publish( &ready_next );
          }
          else{
            ready_next.data = false;
            error_check.publish( &ready_next );
          }
      squareError_msg1.data[0] = m3a.errorPos;
      squareError_msg1.data[1] = m3b.errorPos; //first test, read encoder for all three, c was not responsive and A and B kept flickering back and forth.
      squareError_msg1.data[2] = m3c.errorPos; 

//      squareError_msg1.data[0] = squareError1[0];
//      squareError_msg2.data[0] = squareError2[0];
//      squareError_msg3.data[0] = squareError3[0];
      
      squareErrorPub1.publish( &squareError_msg1 );
      //squareErrorPub2.publish( &squareError_msg2 );
      //squareErrorPub3.publish( &squareError_msg3 );
      ArduinoInterface.spinOnce();
      delay(50);
  
}
////    if (ROS_switch){ //Error Switch is set to individual motors
////      squareError_msg1.data[0] = 0; //squareError1[1];
////      squareError_msg1.data[1] = 2200; //squareError1[2];
////      squareError_msg1.data[2] = 3300; //squareError1[3];
//      //squareError_msg1.data[3] = squareError1[3];
//
////      squareError_msg2.data[0] = squareError2[1];
////      squareError_msg2.data[1] = squareError2[2];
////      squareError_msg2.data[2] = squareError2[3];
//
////      squareError_msg3.data[0] = squareError3[1];
////      squareError_msg3.data[1] = squareError3[2];
////      squareError_msg3.data[2] = squareError3[3];
//      
//      squareErrorPub1.publish( &squareError_msg1 );
//      //squareErrorPub2.publish( &squareError_msg2 );
//      //squareErrorPub3.publish( &squareError_msg3 );
//      ArduinoInterface.spinOnce();
//      delay(100);
//    }
  
    else{
    m3c.log_on_off();
    }
  }


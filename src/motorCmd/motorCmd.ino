

//#include "motorClass.h"
#include <SPI.h>
#include "Encoder.h"
//#include "ros.h"



//void calibrate(motorClass m1,motorClass m2,motorClass m3){
//  float spd = 0.1;
//  while(digitalRead(m1.getLimit()) || digitalRead(m2.getLimit()) || digitalRead(m3.getLimit()) ){
//    
//    if( digitalRead(m1.getLimit()) ){m1.setMotorVel(spd);}
//    else{
//      m1.setMotorVel(0);
//      m1.clearEncoder();
//      }
//    if( digitalRead(m2.getLimit()) ){m2.setMotorVel(spd);}
//    else{
//      m2.setMotorVel(0);
//      m2.clearEncoder();
//    }
//    if( digitalRead(m3.getLimit()) ){m3.setMotorVel(spd);}
//    else{
//      m3.setMotorVel(0);
//      m3.clearEncoder();
//    }


    //m1.vel_closedLoopController();
    //m2.vel_closedLoopController();
    //m3.vel_closedLoopController();
    //}

    
  
  //}

  //stage 1
  //motorClass m1a =  motorClass(7,6,46,49);
  //motorClass m1b =  motorClass(1,2,3,45);
  //motorClass m1c =  motorClass(1,2,3,47);

  //stage 2
  //motorClass m2a =  motorClass(1,2,3,41);
  //motorClass m2b =  motorClass(1,2,3,39);
  //motorClass m2c =  motorClass(1,2,3,43);

  //stage 3
  //motorClass m3a =  motorClass(1,2,3,25);
  //motorClass m3b =  motorClass(1,2,3,23);
  //motorClass m3c =  motorClass(1,2,3,27);

void setup () {
  Serial.begin(9600);
    //calibrate(m1a,m1b,m1c);
    //calibrate(m2a,m2b,m2c);
    //calibrate(m3a,m3b,m3c);

  initEncoder(49);
}


void loop (){
  //Serial.println(m3a.readEnc());
  Serial.println("helo");
}




#include "motorClass.h"
#include "Arduino.h"


void forward(int dirPin,int pwmPin, int pwmVal){
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin,pwmVal);
}
void backward(int dirPin,int pwmPin, int pwmVal){
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin,pwmVal);
}

void motorClass::pos_on_off_controller(void){
  motor_position_calc();
  errorPos = desiredMotorPos - MotorPos;
  if (desiredMotorPos < 0.1){
    tol = 0.2;
   }
   else{
    tol = on_off_tolerance;
   }
  if(errorPos>(2*tol)){
    forward(_dirPin,_pwmPin,175);
   }
  else if((errorPos>tol) && (errorPos<(2*tol))){
    forward(_dirPin,_pwmPin,85);
   }   
   else if(errorPos<-(2*tol)){
    backward(_dirPin,_pwmPin,175);
   }
   else if((errorPos<-tol) && (errorPos>-(2*tol))){
    backward(_dirPin,_pwmPin,85);
   }
   else{ motorClass::stopMotor();}
}



void motorClass::log_on_off(){
  Serial.print(" MP: "+(String)MotorPos);
  Serial.println(" Error_Pos: "+(String)errorPos);
}

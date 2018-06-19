#include "motorClass.h"
#include "Arduino.h"

void forward(int dirPin,int pwmPin){
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin,255);
}
void backward(int dirPin,int pwmPin){
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin,255);
}

void motorClass::pos_on_off_controller(void){
  if(desiredMotorPos+0.5>MotorPos){
    forward(_dirPin,_pwmPin);
   }
   else if(desiredMotorPos-0.5<MotorPos){
    backward(_dirPin,_pwmPin);
   }
   else{ motorClass::stopMotor();}
}



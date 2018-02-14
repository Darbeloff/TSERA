#include "Arduino.h"
#include "motorClass.h"


///////////////////////Helper Functions///////////////////////  

motorClass::motorClass(int pwmPin,int dirPin){
  _pwmPin = pwmPin;
  _dirPin = dirPin;
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
}


void motorClass::setMotorVel(float vel){
  desiredMotorVel = vel;
}


void motorClass::storeOldVals(void){
  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
}


void motorClass::calc_t(){
  currentTime = millis();
  dt = currentTime - prevTime;
}


float motorClass::motor_velocity_calc(void){
  calc_t();
  MotorVel = (encodercount-encodercountPrev) * dt;
  storeOldVals();
  return MotorVel; 
}


float motorClass::proportional_control(void){
  errorVel = desiredMotorVel - MotorVel;
  pCommand = Kpv * errorVel;
  return pCommand;
}


float motorClass::derivative_control(void){
  calc_t();
  dCommand = Kdv * (errorVel - errorVelPrev) / dt;
  return dCommand;
}


float motorClass::integral_control(void){
  calc_t();
  integratedVelError = integratedVelError + errorVel*dt;
  
  iCommand = Kiv*integratedVelError;

  //deal with integral windup
  if ( iCommand > MAX_PWM ){
    iCommand = MAX_PWM;
    integratedVelError = 0; //subject to change
  }
  else if ( iCommand < MIN_PWM ){
    iCommand = MIN_PWM;
    integratedVelError = 0;
  }
  return iCommand;
}


int motorClass::closedLoopController(void){
  currentCommand = proportional_control() + derivative_control() + integral_control();

  currentCommand = map(desiredMotorVel, -100, 100, -255, 255);
  if (currentCommand < -0.001) {
    digitalWrite(_dirPin, LOW);
  }
  else if(currentCommand > 0.001){
    digitalWrite(_dirPin, HIGH);
  }
  analogWrite(_pwmPin, abs(currentCommand));
  return currentCommand; 
}


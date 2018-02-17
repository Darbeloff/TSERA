#include "Arduino.h"
#include "motorClass.h"



motorClass::motorClass(int pwmPin,int dirPin, int limitPin, int encPin){
  _pwmPin = pwmPin;
  _dirPin = dirPin;
  _limitPin = limitPin;
  _encPin = encPin;
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_encPin, OUTPUT);
  pinMode(_limitPin, INPUT_PULLUP);
}


void motorClass::setMotorVel(float vel){
  desiredMotorVel = vel;
}

int motorClass::getLimit(){
  return _limitPin;
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


float motorClass::vel_proportional_control(void){
  errorVel = desiredMotorVel - MotorVel;
  pCommandv = Kpv * errorVel;
  return pCommandv;
}


float motorClass::vel_derivative_control(void){
  calc_t();
  dCommandv = Kdv * (errorVel - errorVelPrev) / dt;
  return dCommandv;
}


float motorClass::vel_integral_control(void){
  calc_t();
  integratedVelError = integratedVelError + errorVel*dt;
  
  iCommandv = Kiv*integratedVelError;

  //deal with integral windup
  if ( iCommandv > MAX_PWM ){
    iCommandv = MAX_PWM;
    integratedVelError = 0; //subject to change
  }
  else if ( iCommandv < MIN_PWM ){
    iCommandv = MIN_PWM;
    integratedVelError = 0;
  }
  return iCommandv;
}


int motorClass::vel_closedLoopController(void){
  currentCommandv = vel_proportional_control() + vel_derivative_control() + vel_integral_control();

  currentCommandv = map(desiredMotorVel, -100, 100, -255, 255);
  if (currentCommandv < -0.001) {
    digitalWrite(_dirPin, LOW);
  }
  else if(currentCommandv > 0.001){
    digitalWrite(_dirPin, HIGH);
  }
  analogWrite(_pwmPin, abs(currentCommandv));
  return currentCommandv; 
}


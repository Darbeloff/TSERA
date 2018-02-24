#include "Arduino.h"
#include "motorClass.h"
#include <SPI.h>
#include "Encoder.h"



motorClass::motorClass(int pwmPin,int dirPin, int limitPin, int encPin){
  _pwmPin = pwmPin;
  _dirPin = dirPin;
  _limitPin = limitPin;
  _encPin = encPin;
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_encPin, OUTPUT);
  pinMode(_limitPin, INPUT_PULLUP);

  initEncoder(_encPin);
  clearEncoderCount(_encPin);
}

//set velocity and position
void motorClass::setMotorVel(float vel){
  desiredMotorVel = vel;
}

void motorClass::setMotorPos(float pos){
  desiredMotorPos = pos;
}

int motorClass::getLimit(){
  return _limitPin;
  }

/////////////////////////////////ENCODER\\\\\\\\\\\\\\\\\\\\\\\\\

void motorClass::clearEncoder(void){
  clearEncoderCount(_encPin);
  }

signed long motorClass::readEnc(void){
  encodercount = readEncoder(_encPin);
  return encodercount;
  }


////////////////////////////////MISC\\\\\\\\\\\\\\\\\\\\\\\\

void motorClass::storeOldVals(void){
  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
}


void motorClass::calc_t(){
  currentTime = millis();
  dt = currentTime - prevTime;
}



////////////position controller functions!\\\\\\\\\\\\\\\\\

float motorClass::motor_position_calc(void){
  calc_t();
  MotorPos = 1000*(gearRatio/encCntsRev)*(encodercount);
  storeOldVals();
  return MotorPos; 
}

float motorClass::pos_proportional_control(void){
  errorPos = desiredMotorPos - MotorPos;
  pCommandp = Kpp * errorPos;
  return pCommandp;
}


float motorClass::pos_derivative_control(void){
  calc_t();
  dCommandp = Kdp * (errorPos - errorPosPrev) / dt;
  return dCommandp;
}


float motorClass::pos_integral_control(void){
  calc_t();
  integratedPosError = integratedPosError + errorPos*dt;
  
  iCommandp = Kip*integratedPosError;

  //deal with integral windup
  if ( iCommandp > MAX_PWM ){
    iCommandp = MAX_PWM;
    integratedPosError = 0; //subject to change
  }
  else if ( iCommandp < MIN_PWM ){
    iCommandp = MIN_PWM;
    integratedPosError = 0;
  }
  return iCommandp;
}


int motorClass::pos_closedLoopController(void){
  motor_position_calc();

  currentCommandp = pos_proportional_control() + pos_derivative_control() + pos_integral_control();

  currentCommandp = map(desiredMotorPos, -1000, 1000, -255, 255);
  if (currentCommandp < -0.001) {
    digitalWrite(_dirPin, LOW);
  }
  else if(currentCommandp > 0.001){
    digitalWrite(_dirPin, HIGH);
  }
  analogWrite(_pwmPin, abs(currentCommandp));
  return currentCommandp; 
}


////////////velocity controller functions!\\\\\\\\\\\\\\\\\

float motorClass::motor_velocity_calc(void){
  calc_t();
  MotorVel = 1000*(gearRatio/encCntsRev)*(encodercount-encodercountPrev) / dt;
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
  motor_velocity_calc();

  currentCommandv = vel_proportional_control() + vel_derivative_control() + vel_integral_control();

  currentCommandv = map(desiredMotorVel, -1000, 1000, -255, 255);
  if (currentCommandv < -0.001) {
    digitalWrite(_dirPin, LOW);
  }
  else if(currentCommandv > 0.001){
    digitalWrite(_dirPin, HIGH);
  }
  analogWrite(_pwmPin, abs(currentCommandv));
  return currentCommandv; 
}


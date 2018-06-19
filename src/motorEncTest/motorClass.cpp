#include "Arduino.h"
#include "motorClass.h"
#include "Encoder.h"


//Constructor
motorClass::motorClass(int pwmPin,int dirPin, int limitPin, int encPin,float gearRatio, float encCntsRev){
  _pwmPin = pwmPin;
  _dirPin = dirPin;
  _limitPin = limitPin;
  _encPin = encPin;
  _gearRatio = gearRatio;
  _encCntsRev = encCntsRev;
  
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


//return limit
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
  calc_t();
  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
  errorPosPrev = errorPos;
}


void motorClass::calc_t(){
  currentTime = micros();
  dt = (currentTime - prevTime)/1000000.0;
}



////////////position controller functions!\\\\\\\\\\\\\\\\\

float motorClass::motor_position_calc(void){
  calc_t();
  encodercount = readEnc();
  MotorPos = (1/_gearRatio)*(1/_encCntsRev)*(encodercount); //revolutions of the output shaft
  storeOldVals();
  return MotorPos; 
}

float motorClass::pos_proportional_control(void){
  //errorPos = desiredMotorPos - MotorPos;
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
  integratedPosError = integratedPosError + errorPos;
  iCommandp = Kip*integratedPosError;
  return iCommandp;
}


int motorClass::pos_closedLoopController(void){
  motor_position_calc();
  errorPos = desiredMotorPos - MotorPos;
//  if (abs(errorPos)<0.005){
//    errorPos = 0;
//    }
  
  currentCommandp = pos_proportional_control() + pos_derivative_control() + pos_integral_control();

  currentCommandp = constrain(map(currentCommandp, -1000, 1000, -255, 255),-255,255);

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
  encodercount = readEnc();
  MotorVel = (1/_gearRatio)*(1/_encCntsRev)*(encodercount-encodercountPrev)/dt;//motor shaft revolutions per second
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
  errorVel = desiredMotorVel - MotorVel;
  integratedVelError = integratedVelError + errorVel;
  
  iCommandv = Kiv*integratedVelError;
  return iCommandv;
}


void motorClass::stopMotor(void){
  analogWrite(_pwmPin,0);
}


int motorClass::vel_closedLoopController(void){
  motor_velocity_calc();

  float Pv=vel_proportional_control();
  float Dv=vel_derivative_control();
  float Iv=vel_integral_control();

  currentCommandv = Pv+Iv+Dv;

  currentCommandv = constrain(map(currentCommandv, -1000, 1000, -255, 255),-255,255);
  if (currentCommandv < -0.001) {
    digitalWrite(_dirPin, HIGH);
  }
  else if(currentCommandv > 0.001){
    digitalWrite(_dirPin, LOW);
  }
  analogWrite(_pwmPin,abs(currentCommandv));
  return currentCommandv; 
}

void motorClass::logValues(void) {
  Serial.print(" MP: "+(String)MotorPos);
  Serial.print(" ErrorPos: "+ (String)errorPos);
  Serial.print(" integrated Error: "+ (String)(integratedPosError));
  Serial.print(" integrated CMD: "+ (String)(iCommandp));
  Serial.println(" MotorCommand: " + (String)currentCommandp);

}


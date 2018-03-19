void calibrate(motorClass m1,motorClass m2,motorClass m3){
  float spd = -0.3;
  while(  !digitalRead(m1.getLimit()) || !digitalRead(m2.getLimit()) || !digitalRead(m3.getLimit()) ){
    
    if( !digitalRead(m1.getLimit() )){
      m1.setMotorVel(spd);
      m1.vel_closedLoopController();
    }
    else{
      m1.setMotorVel(0);
      m1.clearEncoder();
      m1.stopMotor();
      }
    if( !digitalRead(m2.getLimit()) ){
      m2.setMotorVel(spd);
      m2.vel_closedLoopController();
      }
    else{
      m2.setMotorVel(0);
      m2.clearEncoder();
      m2.stopMotor();
    }
    if( !digitalRead(m3.getLimit()) ){
      m3.setMotorVel(spd);
      m3.vel_closedLoopController();}
    else{
      m3.setMotorVel(0);
      m3.clearEncoder();
      m3.stopMotor();
    }


    Serial.print(digitalRead(m1.getLimit()));
    Serial.print(digitalRead(m2.getLimit()));
    Serial.println(digitalRead(m3.getLimit()));

  }
  
  Serial.print(digitalRead(m1.getLimit()));
  Serial.print(digitalRead(m2.getLimit()));
  Serial.println(digitalRead(m3.getLimit()));

  m1.stopMotor();
  m1.clearEncoder();
  m1.setMotorVel(0);

  m2.stopMotor();
  m2.clearEncoder();
  m2.setMotorVel(0);

  m3.stopMotor();
  m3.clearEncoder();
  m3.setMotorVel(0);
}

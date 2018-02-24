#ifndef motorClass_h
#define motorClass_h


class motorClass{
public:
    motorClass(int pwmPin, int dirPin, int limitPin, int encPin); 
    void setMotorVel(float vel);
    void setMotorPos(float pos);
    int getLimit(void);
    void clearEncoder(void);
    signed long readEnc(void);
    int vel_closedLoopController(void);
    int pos_closedLoopController(void);

private:
    //pins
    int _pwmPin;
    int _dirPin;
    int _limitPin;
    int _encPin;
    
    //timing
    unsigned long currentTime = 0;
    unsigned long prevTime = 0;
    float dt = 0; 

    //upkeep functions
    void storeOldVals(void);
    float motor_velocity_calc(void);
    float motor_position_calc(void);
    void calc_t(void);

    //Encoder
    signed long encodercount = 0;
    signed long encodercountPrev = 0;



    //error vel
    float errorVel = 0;
    float errorVelPrev = 0;
    float integratedVelError = 0;

    //velocity control efforts
    float pCommandv = 0;
    float dCommandv = 0;
    float iCommandv = 0;
    int currentCommandv = 0;


    //error pos
    float errorPos = 0;
    float errorPosPrev = 0;
    float integratedPosError = 0;

    //velocity control efforts
    float pCommandp = 0;
    float dCommandp = 0;
    float iCommandp = 0;
    int currentCommandp = 0;
    
    //Position Setpoint and state
    float MotorPos;
    float desiredMotorPos = 0;

    //velocity controller gains
    float Kpp = 1;
    float Kdp = 0;
    float Kip = 0;
    
    //Velocity Setpoint and state
    float MotorVel;
    float desiredMotorVel = 0;
    float avgMotorVel;

    //velocity controller gains
    float Kpv = 1;
    float Kdv = 0;
    float Kiv = 0;

    //velocity controller functions
    float vel_proportional_control(void);
    float vel_derivative_control(void);
    float vel_integral_control(void);




    //position controller functions
    float pos_proportional_control(void);
    float pos_derivative_control(void);
    float pos_integral_control(void);
    

};


//Constants
const int MAX_PWM = 1000;
const int MIN_PWM = -1000;

const float gearRatio = 298.0;
const float encCntsRev = 12.0;

#endif

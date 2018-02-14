#ifndef motorClass_h
#define motorClass_h


class motorClass{
public:
    motorClass(int pwmPin, int dirPin); 
    void setMotorVel(float vel);
    signed long encodercount = 0;


private:
    int _pwmPin;
    int _dirPin;
    unsigned long currentTime = 0;
    unsigned long prevTime = 0;
    float dt = 0; 
    signed long encodercountPrev = 0;
    float errorVel = 0;
    float errorVelPrev = 0;
    float integratedVelError = 0;
    float pCommand = 0;
    float dCommand = 0;
    float iCommand = 0;
    int currentCommand = 0;
    float MotorVel;
    float desiredMotorVel = 0;
    float avgMotorVel;

//controller gains
    float Kpv = 0;
    float Kdv = 0;
    float Kiv = 0;


    void storeOldVals(void);
    float motor_velocity_calc(void);
    void calc_t(void);


//controllers
    float proportional_control(void);
    float derivative_control(void);
    float integral_control(void);
    int closedLoopController(void);

};


//Constants
const int MAX_PWM = 255;
const int MIN_PWM = 0;

#endif

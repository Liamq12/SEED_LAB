#include "robot.h"

#define KDriftReducer 20    // 16
#define FORWARD_KP 0.05
#define FORWARD_KI 0.1              // 0.05
#define KDrift 3.5          // 3.5
#define I_MAX 0.7           // 0.5
#define MAX_PWM 150

void Robot::goForward(){
    long leftPos = leftMotor.getPosition();
    long rightPos = rightMotor.getPosition();

    long drift = leftPos - rightPos;

    long leftCompensation = 0;
    long rightCompensation = 0;

    if(drift > 0){
        leftCompensation = (abs(drift) < KDriftReducer) ? 0 : abs(drift);
        rightCompensation = 0;
    } else if(drift < 0){
        leftCompensation = 0;
        rightCompensation = (abs(drift) < KDriftReducer) ? 0 : abs(drift);
    }

    // Serial.println(String(leftCompensation) + "," + String(rightCompensation) + "," + String(drift));

    integralError += distanceError * (LOOP_DELAY / 1000.); 
    integralError = constrain(integralError, -I_MAX * (LOOP_DELAY / 1000), I_MAX * (LOOP_DELAY / 1000));

    int16_t leftPWM = min((distanceError * FORWARD_KP) + (FORWARD_KI * integralError) - (KDrift * leftCompensation), MAX_PWM);
    int16_t rightPWM = min((distanceError * FORWARD_KP) + (FORWARD_KI * integralError) - (KDrift * rightCompensation), MAX_PWM);

    leftMotor.setPWM(leftPWM);
    rightMotor.setPWM(rightPWM);

    // Serial.print("Inline: " + String(inlineError,4) + ", Normal: " + String(normalError,4));
    // Serial.println(", Inline PWM: " + String(inline_PWM,4) + ", Normal PWM: " + String(normal_PWM,4) + ", Normal I PWM: " + String(normalIntegral * Ki_normal,4));  
}

void Robot::turnTo(){
    long leftPos = leftMotor.getPosition();
    long rightPos = rightMotor.getPosition();

    long drift = leftPos + rightPos;

    long leftCompensation = 0;
    long rightCompensation = 0;

    if(drift > 0){
        leftCompensation = (abs(drift) < KDriftReducer) ? 0 : abs(drift);
        rightCompensation = 0;
    } else if(drift < 0){
        leftCompensation = 0;
        rightCompensation = (abs(drift) < KDriftReducer) ? 0 : abs(drift);
    }

    // Serial.println(String(leftCompensation) + "," + String(rightCompensation) + "," + String(drift));

    integralError += distanceError * (LOOP_DELAY / 1000.); 
    integralError = constrain(integralError, -I_MAX * (LOOP_DELAY / 1000), I_MAX * (LOOP_DELAY / 1000));

    int16_t leftPWM = min((distanceError * FORWARD_KP) + (FORWARD_KI * integralError) - (KDrift * leftCompensation), MAX_PWM);
    int16_t rightPWM = min((distanceError * FORWARD_KP) + (FORWARD_KI * integralError) - (KDrift * rightCompensation), MAX_PWM);

    leftMotor.setPWM(leftPWM);
    rightMotor.setPWM(rightPWM);

    // Serial.print("Inline: " + String(inlineError,4) + ", Normal: " + String(normalError,4));
    // Serial.println(", Inline PWM: " + String(inline_PWM,4) + ", Normal PWM: " + String(normal_PWM,4) + ", Normal I PWM: " + String(normalIntegral * Ki_normal,4));  
    
}

void Robot::positionController(){
    // Calculate current position

    if(abs(angleError) > 0.174533 && mode == RobotMode::FORWARD){
        mode = RobotMode::TURN;
        leftMotor.clearPosition();
        rightMotor.clearPosition();
    } else if (abs(angleError) < 0.020 && mode == RobotMode::TURN){
        mode = RobotMode::FORWARD;
        leftMotor.setPWM(0);
        rightMotor.setPWM(0);
        delay(1000);
        leftMotor.clearPosition();
        rightMotor.clearPosition();
    }
    
    if(mode == RobotMode::FORWARD){
        goForward(); // Move towards target / station keep
    } else if(mode == RobotMode::TURN){
        turnTo(); // Angle is too far off rotate bot
    } else if(mode == RobotMode::LEFT90){
        leftMotor.setPWM(50);
        rightMotor.setPWM(-50);
        delay(1000);
        leftMotor.setPWM(0);
        rightMotor.setPWM(0);
        while(true){delay(100);}
    } else if(mode == RobotMode::RIGHT90){
        leftMotor.setPWM(-50);
        rightMotor.setPWM(50);
        delay(1000);
        leftMotor.setPWM(0);
        rightMotor.setPWM(0);
        while(true){delay(100);}
    } 
}

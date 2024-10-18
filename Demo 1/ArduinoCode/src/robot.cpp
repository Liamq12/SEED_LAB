#include "robot.h"

Position::Position(double p_x, double p_y, double p_phi){
    x = p_x;
    y = p_y;
    phi = p_phi;
}

// Function to calculate Euclidean distance between two points (x1, y1) and (x2, y2)
double Robot::getEuclideanDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Function to calculate the perpendicular distance from point (x3, y3) 
// to the line defined by points (x1, y1) and (x2, y2)
double Robot::getNormalDistance(double x1, double y1, double x2, double y2, double x3, double y3) {
    double A = y2 - y1;
    double B = x1 - x2;
    double C = x2 * y1 - x1 * y2;

    return (-1)*(A * x3 + B * y3 + C) / sqrt(A * A + B * B);
}

void Robot::calcDistanceError(Position& startPosition, Position& currentPosition, Position& targetPosition) {
    // Calculate the normal error (perpendicular distance to the line from start to target)
    normalError = getNormalDistance(startPosition.x, startPosition.y,
                                    targetPosition.x, targetPosition.y,
                                    currentPosition.x, currentPosition.y);    

    // Calculate the distance error (Euclidean distance to the target)
    double distance_to_target = getEuclideanDistance(targetPosition.x, targetPosition.y, currentPosition.x, currentPosition.y);

    // Inline distance calculation (distance along the line to the target)
    inlineError = sqrt(pow(distance_to_target, 2) - pow(normalError, 2));
}

void Robot::setTargetPosition(Position& p_targetPosition){
    startPosition = currentPosition; // Start to current position

    double deltaX = p_targetPosition.x - currentPosition.x;
    double deltaY = p_targetPosition.y - currentPosition.y;

    // atan2 returns the angle in radians
    double angleRadians = atan2(deltaY, deltaX);

    // Normalize the angle to be within [0, 360)
    if (angleRadians < 0) {
        angleRadians += 2*PI;
    }

    // Transfer Calculated Values to target position
    targetPosition.x = p_targetPosition.x;
    targetPosition.y = p_targetPosition.y;
    targetPosition.phi = angleRadians;
}

void Robot::calculatePosition(){
    // Find Count difference
    long diffLeftCount = leftMotor.getPosition() - previousLeftMotorPosition;
    long diffRightCount = rightMotor.getPosition() - previousRightMotorPosition;

    // Update previous Count
    previousLeftMotorPosition = leftMotor.getPosition();
    previousRightMotorPosition = rightMotor.getPosition(); // Flip direction of right motor

    // Calculate movement in meters
    double dl = ((double)diffLeftCount / COUNTS_PER_REV) * METERS_PER_REV;
    double dr = ((double)diffRightCount / COUNTS_PER_REV) * METERS_PER_REV;

    // Calculate update position
    currentPosition.x += cos(currentPosition.phi) * ((dl + dr) / 2);
    currentPosition.y += sin(currentPosition.phi) * ((dl + dr) / 2);

    currentPosition.phi += (1 / WHEEL_WIDTH) * (dl - dr);
}

void Robot::goToPosition(double inlineError, double normalError){
    double Kp_inline = 200, Kp_normal = 400;
    double inlineMax = 60, normalMax = 30;

    double Ki_inline = 0, Ki_normal = 0;
    double integralThreshold = 0.5;

    // Inline Proportional
    double inline_PWM = 0;
    if(inlineMax < fabs(inlineError*Kp_inline)){
        if(fabs(inlineError*Kp_inline) > 0)
            inline_PWM = inlineMax;
        else
            inline_PWM = -inlineMax;
    } else {
        inline_PWM = inlineError*Kp_inline;
    }

    // Inline Integral
    if(fabs(inlineError) < integralThreshold){
       inlineIntegral += inlineError;
    } else {
        inlineIntegral = 0;
    }
    inline_PWM += inlineIntegral * Ki_inline;

    // Normal Proportional
    double normal_PWM = 0;
    if(normalMax < fabs(normalError*Kp_normal)){
        if(fabs(normalError*Kp_normal) > 0)
            normal_PWM = normalMax;
        else
            normal_PWM = -normalMax;
    } else {
        normal_PWM = normalError*Kp_normal;
    }

    // Normal Integral
    if(fabs(normalError) < integralThreshold){
       normalIntegral += normalError;
    } else {
        normalIntegral = 0;
    }
    normal_PWM += normalIntegral * Ki_normal;

    // Motor Control
    Serial.print("Inline: " + String(inlineError) + ", Normal: " + String(normalError));
    Serial.println(", Inline PWM: " + String(inline_PWM) + ", Normal PWM: " + String(normal_PWM));        
    leftMotor.setVoltage(inline_PWM + normal_PWM);
    rightMotor.setVoltage(inline_PWM - normal_PWM);    
}

void Robot::turnTo(double angleError){
    double Kp_turn = 200, Ki_turn = 1;
    double turnMax = 40;
    double integralThreshold = 0.5;

    // P
    double angle = 0;
    if(turnMax < fabs(angleError*Kp_turn)){
        if(fabs(angleError*Kp_turn) > 0)
            angle = turnMax;
        else
            angle = -turnMax;
    } else {
        angle = angleError*Kp_turn;
    }

    // I
    if(fabs(angleError) < integralThreshold){
       angleIntegral +=  angleError;
    } else {
        angleIntegral = 0;
    }
    angle += angleIntegral * Ki_turn;


    Serial.print("Angle: " + String(angle));
    Serial.println(", Angle PWM: " + String(angle));        
    leftMotor.setVoltage(angle);
    rightMotor.setVoltage(-angle);
    // I

    // D
    
}

void Robot::positionController(){
    // Calculate current position
    calculatePosition();
    Position errorPosition = targetPosition - currentPosition;
    calcDistanceError(startPosition, currentPosition, targetPosition);

    if(abs(errorPosition.phi) < 0.0174533){ // Angle Error is less than +- 1 deg
        goToPosition(inlineError, normalError); // Move towards target / station keep

    } else {
        turnTo(errorPosition.phi); // Angle is too far off rotate bot
    }
}

#include "robot.h"

Position::Position(double p_x, double p_y, double p_phi){
    x = p_x;
    y = p_y;
    phi = p_phi;
}

void Robot::setTargetPosition(Position& p_targetPosition){
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

void Robot::positionController(){
    // Calculate current position
    calculatePosition();
    Position errorPosition = targetPosition - currentPosition;

    Serial.print("Xe: " + String(errorPosition.x) + ", Ye: " + String(errorPosition.y) + ", Phie: " + String(errorPosition.phi) + ", : ");

    if(abs(errorPosition.phi) < 0.0174533){ // Angle Error is less than +- 1 deg
        // Move towards target / station keep
        Serial.println("Moving in straight line");

    } else {
        // Angle is too far off rotate bot
        Serial.println("Rotate");
    }
}

// // Future integration
// void motorController(){
//     int error1 = target1 - rightPos;
//     int error2 = target2 - leftPos;

//     double voltage1 = min((error1 * KP) / REDUCER, BAT_VOLTAGE);
//     double voltage2 = min((error2 * KP) / REDUCER, BAT_VOLTAGE);

  
// }



// Loop
// // Use following code to generate target angle
// // Function to calculate the angle to the target point
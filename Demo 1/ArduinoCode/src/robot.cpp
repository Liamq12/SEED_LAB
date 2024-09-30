#include "robot.h"

Position::Position(double p_x, double p_y, double p_phi){
    x = p_x;
    y = p_y;
    phi = p_phi;
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

    if(abs(errorPosition.phi) < 0.0174533){ // Angle Error is less than +- 1 deg
        // Move towards target / station keep

    } else {
        // Angle is too far off rotate bot
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
// double calculateAngle(const Point& current, const Point& target) {
//     double deltaX = target.x - current.x;
//     double deltaY = target.y - current.y;

//     // atan2 returns the angle in radians
//     double angleRadians = atan2(deltaY, deltaX);

//     // Convert to degrees
//     double angleDegrees = angleRadians * (180.0 / PI); // Use PI defined in Arduino

//     // Normalize the angle to be within [0, 360)
//     if (angleDegrees < 0) {
//         angleDegrees += 360.0;
//     }

//     return angleDegrees;
// }
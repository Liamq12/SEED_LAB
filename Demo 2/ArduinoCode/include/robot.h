#include <Arduino.h>
#include "motor.h"

#ifndef ROBOT_H
#define ROBOT_H


#define REDUCER 4
#define COUNTS_PER_REV 3200
#define COUNTS_PER_RAD 509
#define METERS_PER_REV 0.4738
#define WHEEL_WIDTH 0.2745

#define LOOP_DELAY 1500

enum RobotMode {
    TURN,
    FORWARD
};

class Robot {
private:
    // Setup Motors
    MotorShield motorShield{};
    Motor& leftMotor = motorShield.getLeftMotor();
    Motor& rightMotor = motorShield.getRightMotor();

    // Private constructor to prevent instantiation
    Robot(){}

    // Static pointer to hold the single instance
    static Robot* instance;

    long previousLeftMotorPosition = 0;
    long previousRightMotorPosition = 0;

    double integralError = 0;

public:
    // Deleted copy constructor and assignment operator
    Robot(Robot const&) = delete;
    void operator=(Robot const&) = delete;

    // Method to access the singleton instance
    static Robot* getInstance() {
        if (instance == nullptr) {
            instance = new Robot();
        }
        return instance;
    }

    RobotMode mode = RobotMode::FORWARD;

    double distanceError = 0;
    double angleError = 0;

    void setError();

    void positionController();

    void goForward();

    void turnTo();
};


#endif // ROBOT_H
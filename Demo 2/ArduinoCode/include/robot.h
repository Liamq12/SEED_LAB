#include <Arduino.h>
#include "motor.h"

#ifndef ROBOT_H
#define ROBOT_H

#define KP 0.05
#define REDUCER 4
#define COUNTS_PER_REV 3200
#define COUNTS_PER_RAD 509
#define METERS_PER_REV 0.4738
#define WHEEL_WIDTH 0.2745

enum RobotMode {
    TURN,
    GO_TO
};

struct Position { // Units are in mm
    double x = 0;
    double y = 0;
    double phi = 0;

    Position() {}
    Position(double p_x, double p_y, double p_phi);

    // Overload the subtraction operator
    Position operator-(const Position& other) const {
        Position result;
        result.x = this->x - other.x;
        result.y = this->y - other.y;
        result.phi = this->phi - other.phi;
        return result;
    }
};

class Robot {
private:
    // Setup Motors
    MotorShield motorShield{};
    Motor& leftMotor = motorShield.getLeftMotor();
    Motor& rightMotor = motorShield.getRightMotor();

    // Private constructor to prevent instantiation
    Robot():currentPosition(0,0,0),targetPosition(0,0,0) {}

    // Static pointer to hold the single instance
    static Robot* instance;

    long previousLeftMotorPosition = 0;
    long previousRightMotorPosition = 0;

    double inlineError = 0;
    double normalError = 0;

    double inlineIntegral = 0;
    double normalIntegral = 0;

    double angleIntegral = 0;

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

    // Position Variables & Functions
    Position startPosition;
    Position currentPosition;
    Position targetPosition;

    RobotMode mode = RobotMode::GO_TO;

    void calculatePosition();

    void setTargetPosition(Position& p_targetPosition);
    void setTargetAngle(double targetAngle);

    Position& getPosition(){ return currentPosition; }

    double getEuclideanDistance(double x1, double y1, double x2, double y2);
    double getNormalDistance(double x1, double y1, double x2, double y2, double x3, double y3);
    void calcDistanceError(Position& startPosition, Position& currentPosition, Position& targetPosition);

    void positionController();

    void goToPosition(double inlineError, double normalError);
    void turnTo(double angleError);
};


#endif // ROBOT_H
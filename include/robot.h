#pragma once
#include "main.h"
#include "odometry.h"
#include "pid.h"

class Robot
{
public:
    void SplitArcade(int dir, int turn);
    void init();

    // Lift
    void SetLiftHeight(float height);

    // Odometery
    void HeadingUpdateLoop();
    void TurnHeading(double targetAngle);
    Position GetPosition();

private:
    // Lift
    pros::Motor liftMotor = pros::Motor(16, pros::MotorGears::red, pros::MotorUnits::degrees);
    //! Set to acuall values
    const float StartingLiftAngle = 25.5f;
    const float TowerHeight = 17.5f;
    const float midStageHeight = 10;
    const float armLength = 16;

    // Drive
    bool driveDisabled = false;
    bool traingMode = true;
    PID turnPID = PID(50, 0, 0, 0);
    pros::MotorGroup leftDrive = pros::MotorGroup({-6, 8, 10});
    pros::MotorGroup rightDrive = pros::MotorGroup({5, 13, -15});
    Odometry odm = Odometry(11, 18, 3, 20, 4, 1);
};
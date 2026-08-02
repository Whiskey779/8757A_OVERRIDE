#include "robot.h"
#include "pid.h"
#include <cmath>
#include "angle.h"

#define turningErrorThreshold 1

void Robot::SplitArcade(int dir, int turn)
{
    if (traingMode)
    {
        if (!driveDisabled)
        {
            if (std::abs(dir) > 114 || std::abs(turn) > 114)
            {
                driveDisabled = true;
                leftDrive.move(0);
                rightDrive.move(0);
            }
            else
            {
                leftDrive.move(dir + turn);
                rightDrive.move(dir - turn);
            }
        }
        else if (dir == 0 && turn == 0)
            driveDisabled = false;
    }
    else
    {
        leftDrive.move(dir + turn);
        rightDrive.move(dir - turn);
    }
}

void Robot::init()
{
    odm.initialise();
}

void Robot::SetLiftHeight(float targetHeight)
{
    double targetMotorPostion = 7 * (std::asin((targetHeight - TowerHeight - midStageHeight) / (2 * armLength)) - StartingLiftAngle);
    if (targetMotorPostion != targetMotorPostion)
    {
        return;
    }
}

Position Robot::GetPosition()
{
    Position relativePos = odm.GetPosition();
    Position absPos;
    absPos.heading = relativePos.heading;
    // absPos.heading = std::atan2(std::sin(relativePos.heading), std::cos(relativePos.heading)) * 180 / M_PI;
    absPos.x = relativePos.x * (relativePos.heading / 2);
    absPos.y = relativePos.y * (relativePos.heading / 2);
    return absPos;
}

void Robot::HeadingUpdateLoop()
{
    while (true)
    {
        odm.updateHeading();
        pros::delay(10);
    }
}

void Robot::TurnHeading(double targetAngleDeg)
{
    double targetAngle = DegToRad(targetAngleDeg);
    Position pos = odm.GetPosition();
    double currentHeading = std::atan2(std::sin(pos.heading), std::cos(pos.heading));
    double diff = calcAngleDif(currentHeading, targetAngle);
    double offset;
    if (diff <= M_PI)
        offset = 0;
    else
    {
        offset = -currentHeading;
        targetAngle = diff;
    }
    double error = targetAngle - currentHeading;
    do
    {
        double speed = turnPID.Calculate(targetAngle, currentHeading + offset);
        speed = std::clamp((int)speed, -127, 127);
        leftDrive.move(speed);
        rightDrive.move(-speed);
        pos = odm.GetPosition();
        currentHeading = std::atan2(std::sin(pos.heading), std::cos(pos.heading));
    } while (error > turningErrorThreshold);
    rightDrive.move(0);
    leftDrive.move(0);
}
#include "pid.h"
#include <cmath>

PID::PID(double Kp, double Ki, double Kd, double integralStart) : Kp(Kp), Ki(Ki), Kd(Kd), integralStart(integralStart) {}

double PID::Calculate(double target, double current)
{
    double error = target - current;
    integral += error;
    if (error == 0 || std::abs(error) > integralStart)
    {
        integral = 0;
    }
    double derivative = error - prevError;
    prevError = error;

    return error * Kp + integral * Ki + derivative * Kd;
}
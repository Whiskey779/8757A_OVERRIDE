#pragma once

class PID
{
public:
    PID(double Kp, double Ki, double Kd, double integralStart);
    double Calculate(double target, double current);

private:
    double Ki, Kp, Kd, integralStart;
    double errorThreshold;
    double prevError = 0, integral = 0;
};
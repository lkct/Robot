#include <algorithm>
#include "control.h"
#include "ProcessorMulti_Processor_Core_Vars.h"

using std::replace;
using std::min_element;

void PIDCtrl::Set(double kp, double ki, double kd)
{
    err_prev = err_intg = 0;
    Kp = kp;
    Ki = ki;
    Kd = kd;
}
void PIDCtrl::Reset()
{
    err_prev = err_intg = 0;
}
double PIDCtrl::Step(double err)
{
    err_intg += err;
    double delta = Kp * err + Ki * err_intg + Kd * (err - err_prev);
    err_prev = err;
    return delta;
}

int getSteer(double dis, double yaw, int lsrsize, short* lsrdata, double lsrunit,
             ProcessorMulti_Processor_Core_Params* params, ProcessorMulti_Processor_Core_Vars* vars)
{
    int speed = 100;
    int steer = 100;
    double target = 90;
    double err = target - 90;

    replace(lsrdata, lsrdata + lsrsize, 0, params->infDistance);

    double s = 0;
    double w = 1e-6;
    for (int i = 0; i < lsrsize; i++)
    {
        s += i * lsrdata[i];
        w += lsrdata[i];
    }
    target = s * 0.5 / w;

    err = target - 90;
    steer = vars->pid.Step(err);

    if (abs(steer) < params->straightThres)
    {
        speed = params->straightSpeed;
    }

    // convert to return steer
    steer += params->baseSteer;
    int maxSteer = 500;
    if (steer > maxSteer)
    {
        steer = maxSteer;
    }
    if (steer < -maxSteer)
    {
        steer = -maxSteer;
    }

    if (vars->reverse && vars->prev_odom - dis > params->backwardDis / lsrunit)
    {
        vars->reverse = false;
    }
    short *lsrmid = lsrdata + (lsrsize - 1) / 2;
    int safeRange = params->safeAngle * 2;
    if (min_element(lsrmid - safeRange, lsrmid + safeRange + 1) < params->safeDis || vars->reverse)
    {
        speed = -100;
        steer = params->baseSteer;
        if (!vars->reverse)
        {
            vars->reverse = true;
            vars->prev_odom = dis;
        }
    }

    return (speed << 16) + (steer & 0xffff);
}

#include <algorithm>
#include "control.h"
#include "ProcessorMulti_Processor_Core_Vars.h"

using std::nth_element;
using std::max_element;
using std::find;

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

short getSteer(double dis, double yaw, int lsrsize, short* lsrdata,
               ProcessorMulti_Processor_Core_Params* params, ProcessorMulti_Processor_Core_Vars* vars)
{
    short steer = 100;
    double target = 90;
    double err = 90 - target;

    bool useAvg = find(lsrdata, lsrdata + lsrsize, 0) - lsrdata != lsrsize;
    if (useAvg)
    {
        double s = 0;
        double w = 1e-6;
        for (int i = 0; i < lsrsize; i++)
        {
            s += i * 0.5 * lsrdata[i];
            w += lsrdata[i];
        }
        target = s / w;
    }
    else
    {
        int filt = params->filterWindow;
        short *wind = new short[filt];
        short *newdata = new short[lsrsize];
        for (int i = 0; i < lsrsize - filt; i++)
        {
            memcpy(wind, lsrdata + i, filt * sizeof(short));
            nth_element(wind, wind + filt / 2, wind + filt);
            newdata[i + filt / 2] = wind[filt / 2];
        }
        target = (max_element(newdata, newdata + lsrsize) - newdata) * 0.5;
        delete[] wind;
        delete[] newdata;
    }

    err = target - 90;
    qDebug()<<err<<endl;
    steer = vars->pid.Step(err);

    // convert to return value
    steer = -steer; // ???
    steer += params->baseSteer;
    if (steer > 400)
    {
        steer = 400;
    }
    if (steer < -400)
    {
        steer = -400;
    }
    return steer;
}

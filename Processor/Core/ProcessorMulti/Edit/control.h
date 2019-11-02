#ifndef CONTROL_H
#define CONTROL_H

class ProcessorMulti_Processor_Core_Params;
class ProcessorMulti_Processor_Core_Vars;

class PIDCtrl
{
public:
    void Set(double kp, double ki, double kd);
    void Reset();
    double Step(double err);

private:
    double err_prev;
    double err_intg;
    double Kp, Ki, Kd;

};

short getSteer(double dis, double yaw, int lsrsize, short* lsrdata,
               ProcessorMulti_Processor_Core_Params* params, ProcessorMulti_Processor_Core_Vars* vars);

#endif // CONTROL_H

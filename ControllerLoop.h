#pragma once

#include "mbed.h"
#include "EncoderCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "sensors_actuators.h"


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(sensors_actuators *,float Ts);
    virtual     ~ControllerLoop();
    void start_loop(void);
    void enable_vel_cntrl(void);
    void enable_bal_cntrl(void);
    void reset_cntrl(void);
    void disable_all_cntrl();

private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    PID_Cntrl I4, fw_cntrl;
    float Ts,km;
    bool bal_cntrl_enabled;
    bool vel_cntrl_enabled;
    void sendSignal();
    sensors_actuators *m_sa;
    float saturate(float,float,float);
    float K6[6]{-7.1051, -9.7315, -0.6375, -2.2270, 0.0015, -0.0010 };
};

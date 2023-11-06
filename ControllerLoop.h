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
    float K6[6]{5.0697, -5.8136, 0.4605, -1.4806, 0.0003, -0.0001};
};

#pragma once

#include "mbed.h"
#include "EncoderCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "sensors_actuators.h"
#include "IIR_filter.h"


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(sensors_actuators *,float Ts);
    virtual ~ControllerLoop();
    void start_loop(void);
    void enable_vel_cntrl(void);
    void enable_bal_cntrl(void);
    void reset_cntrl(void);
    void disable_all_cntrl();
    float phi_bd_des;
 
private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    PID_Cntrl flat_vel_cntrl;
    PID_Cntrl I_cntrl;
    float Ts;
    float km = 36.9E-3; // Motor constant Nm/Amp
    bool bal_cntrl_enabled;
    bool vel_cntrl_enabled;
    void sendSignal();
    float est_angle();
    sensors_actuators *m_sa;
    float saturate(float,float,float);
};
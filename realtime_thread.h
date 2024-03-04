#pragma once

#include "mbed.h"
#include "EncoderCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "IO_handler.h"


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class realtime_thread
{
public:
    realtime_thread(IO_handler *,float Ts);
    virtual     ~realtime_thread();
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
    PID_Cntrl flat_vel_cntrl;
    PID_Cntrl bal_vel_cntrl;
    float Ts;
    bool bal_cntrl_enabled;
    bool vel_cntrl_enabled;
    void sendSignal();
    float est_angle();
    IO_handler *m_io;
};

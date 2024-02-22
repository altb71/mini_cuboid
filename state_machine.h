#include "mbed.h"
#include "sensors_actuators.h"
#include "ControllerLoop.h"

#define IDLE 0
#define INIT 1
#define BREAK_DISC 2
#define BALANCE 3
#define MISC 4


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,ControllerLoop *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    sensors_actuators *m_sa;
    ControllerLoop *m_loop;
};

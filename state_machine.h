#include "mbed.h"
#include "IO_handler.h"
#include "realtime_thread.h"

#define INIT 1
#define FLAT 2
#define BALANCE 3


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(IO_handler *,realtime_thread *,float Ts);
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
    IO_handler *m_sa;
    realtime_thread *m_loop;
};

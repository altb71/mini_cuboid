#include "state_machine.h"
using namespace std;

// contructor for controller loop
state_machine::state_machine(IO_handler *sa, realtime_thread *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    float local_time;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        local_time = ti.read();
        switch(CS)
            {
            case INIT:
                if(m_sa->get_key_state() && local_time>.5)
                    {
                    printf("switch to FLAT\r\n");
                    CS = FLAT;
                    ti.reset();
                    }
                break;
            case FLAT:
                if(m_sa->get_key_state() && local_time>.5)
                    {
                    printf("switch to BALANCE\r\n");
                    m_loop->enable_bal_cntrl();
                    m_sa->enable_escon();
                    CS = BALANCE;
                    ti.reset();
                    }
                break;
            case BALANCE:
                if(m_sa->get_key_state() && local_time>.5)
                    {
                    printf("switch to INIT\r\n");
                    CS = INIT;
                    m_loop->disable_all_cntrl();
                    m_sa->disable_escon();
                    ti.reset();
                    }
                break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}

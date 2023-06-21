#include "state_machine.h"
using namespace std;

// contructor for controller loop
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096)
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
    
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        switch(CS)
            {
            case INIT:
                if(ti.read()>2)
                    {
                    printf("switch to BREAK_DISC\r\n");
                    CS = BREAK_DISC;
                    ti.reset();
                    }
                break;
            case BREAK_DISC:
                if(fabs(m_sa->get_phi_bd()) < 0.0524 && fabs(m_sa->get_the_bd()) < 0.0524)
                    {
                    printf("switch to BALANCE\r\n");
                    m_loop->enable_bal_cntrl();
                    CS = BALANCE;
                    ti.reset();
                    }
                break;
            case BALANCE:
                if((fabs(m_sa->get_phi_bd()) > 0.0873 || fabs(m_sa->get_the_bd()) > 0.0873) && ti.read()>2)
                    {
                    printf("switch to INIT\r\n");
                    m_loop->disable_all_cntrl();
                    CS = INIT;
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

#include "ControllerLoop.h"
using namespace std;


// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_sa = sa;
    m_sa->disable_escon();
    km = 36.9e-3;
    ti.reset();
    ti.start();

    I4 = PID_Cntrl(0,1,0,0,Ts,-3*fabs(km/K6[5]),3*fabs(km/K6[5]));  // umax = 1 Ampere * km/K6(6)
    fw_cntrl = PID_Cntrl(0.0316,1.58,0,1,Ts,-3*km,3*km);
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){


    float integrator = 0;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_sa->read_sensors_calc_speed();       // first read all sensors, calculate mtor speed
//        float M_soll = 0 - (K2[0] * m_sa->get_phi_bd() +K2[1] * m_sa->get_gz());
       if(bal_cntrl_enabled)
            {
            integrator = I4(0 - m_sa->get_om_fw());
            float M_soll =  -(K6[0] * m_sa->get_phi_bd() + K6[1] * m_sa->get_the_bd() 
                             +K6[2] * m_sa->get_gx()     + K6[3] * m_sa->get_gy() 
                             +K6[4] * m_sa->get_om_fw()  + K6[5] * integrator);
            float i_soll = saturate(M_soll / km,-14,14);
            // -------------------------------------------------------------
            m_sa->write_current(i_soll);                   // write to motor 0 
            m_sa->enable_escon();
            }
        else if(vel_cntrl_enabled)  // true if cube is flat 
            {
            float M_soll = fw_cntrl(0 - m_sa->get_om_fw());
            m_sa->write_current(M_soll/km);   
            m_sa->enable_escon();
                // write to motor 0 
            }
        else {
            m_sa->disable_escon();
            }
        // handle enable
        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}


void ControllerLoop::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
    bal_cntrl_enabled = false;
}
void ControllerLoop::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
    vel_cntrl_enabled = false;
    I4.reset(0);
}
void ControllerLoop::reset_cntrl(void)
{

}
void ControllerLoop::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}
float ControllerLoop::saturate(float v,float mi,float ma)
{
    return (v<mi) ? mi :((v>ma) ? ma : v);
}
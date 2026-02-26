#include "ControllerLoop.h"
#include "GPA.h"
using namespace std;

// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_sa = sa;
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
    I_cntrl.setup(0, 1, 0, 1, Ts, -4, 4);// limits are set: M_max/10/K4[3] = 0.5/10/0.0074
    flat_vel_cntrl.setup(0.0316,1.58,0,1,Ts,-3*km,3*km); // see Matlab code in main
    //----------------------------------------------------------------
    m_sa->disable_escon();
    phi_bd_des = 0;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float i_des = 0;
    uint8_t k = 0;
    float K[2] = {-1.3924, -0.0864};
    float K4[4] = {-2.9527,-0.2872,-0.008,0.0069};
    float M_des;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // ---------------- THE LOOP ---------------------------------------------------------
        m_sa->read_sensors_calc_speed();    // first read all sensors, calculate motor speed
        if(bal_cntrl_enabled)               // for balancing and lift-up
            {
            M_des = -(K4[0]* (m_sa->get_phi_bd()-phi_bd_des) + K4[1]*m_sa->get_gz()     // desired torque
                    +K4[2] * saturate(m_sa->get_vphi_fw(),-50,50) + K4[3] * I_cntrl(0-m_sa->get_vphi_fw()));
            m_sa->write_current(M_des/km);                   // write current to motor
            }
        else if(vel_cntrl_enabled)  // true if cube is flat 
            {
            M_des = flat_vel_cntrl(0 - m_sa->get_vphi_fw());
            m_sa->write_current(M_des/km);                   // write to motor 0 
            }
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
}
void ControllerLoop::reset_cntrl(void)
{
    I_cntrl.reset(0);
    flat_vel_cntrl.reset(0);
}
void ControllerLoop::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}
float ControllerLoop::saturate(float x,float ll, float ul)
{
if(x>ul)
    return ul;
else if(x<ll)
    return ll;
else 
    return x; 
}
#include "realtime_thread.h"
using namespace std;


// contructor for controller loop
realtime_thread::realtime_thread(IO_handler *io, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_io = io;
    m_io->disable_escon();
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void realtime_thread::loop(void){
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_io->read_sensors_calc_speed();       // first read all sensors, calculate motor speed and angle of body
        if(bal_cntrl_enabled)
            {
                /* Aufgabe 5.1 */
            }
        else if(vel_cntrl_enabled)
            {

            }
        else 
            {
            m_io->write_current(0);
            }   
        }// endof the main loop
}

void realtime_thread::sendSignal() {
    thread.flags_set(threadFlag);
}
void realtime_thread::start_loop(void)
{
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), Ts);
}

/* est_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students. 
*/
float realtime_thread::est_angle(void)
{
    return 0;
}

void realtime_thread::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
}
void realtime_thread::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
}
void realtime_thread::reset_cntrl(void)
{

}
void realtime_thread::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}

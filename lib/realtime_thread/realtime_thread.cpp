#include "realtime_thread.h"
using namespace std;

// contructor for controller loop
realtime_thread::realtime_thread(IO_handler *io, float Ts)
    : thread(osPriorityHigh, 4096)
{
    this->Ts = Ts;
    this->m_io = io;
    m_io->disable_escon();
    ti.reset();
    ti.start();
    I_reg = PID_Cntrl(0, 1, 0, 0, Ts, -1000, 1000);
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority

void realtime_thread::loop(void)
{
    float K2[2] = {-0.9602, -0.0611};
    float K4[4] = {-1.7131, -0.1553, -0.0029, 0.0023};
    float M_mot;
    float km = 36.9e-3;
    float kp = 0.5;

    while (1) {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_io->read_sensors_calc_speed(); // first read all sensors, calculate motor speed and angle of body
        if (bal_cntrl_enabled) {
            /* Aufgabe 5.1 */
            // M_mot = -(K2[0] * m_io->get_phi_bd() + K2[1] * m_io->get_gz());
            M_mot = -(K4[0] * m_io->get_phi_bd() + K4[1] * m_io->get_gz() + K4[2] * m_io->get_vphi_fw() +
                      K4[3] * I_reg(0 - m_io->get_vphi_fw()));

            m_io->write_current(M_mot / km);

        } else if (vel_cntrl_enabled) {
            float i_des = kp * (0 - m_io->get_vphi_fw());
            m_io->write_current(i_des);

        } else {
            m_io->write_current(0);
        }
    } // endof the main loop
}

void realtime_thread::sendSignal() { thread.flags_set(threadFlag); }
void realtime_thread::start_loop(void)
{
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), Ts);
}

/* est_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students.
*/
float realtime_thread::est_angle(void) { return 0; }

void realtime_thread::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
    bal_cntrl_enabled = false;
}
void realtime_thread::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
    vel_cntrl_enabled = false;
}
void realtime_thread::reset_cntrl(void) {}
void realtime_thread::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}

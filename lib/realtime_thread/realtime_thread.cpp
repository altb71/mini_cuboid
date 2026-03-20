#include "realtime_thread.h"

#include <chrono>
#include <cstdint>

#include "DataLogger.h"
#include "GPA.h"

extern DataLogger myDataLogger;
extern GPA myGPA;

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

using namespace Eigen;
using namespace std::chrono;

// contructor for realtime_thread loop
realtime_thread::realtime_thread(IO_handler *io_handler, float Ts)
    : m_Thread(osPriorityHigh, 4096)
{
    m_Ts = Ts;                 // the sampling time
    m_IO_handler = io_handler; // a pointer to the io handler
    m_state = INIT;
    m_IO_handler->disable_escon();
    m_Timer.reset();
    m_Timer.start();
    m_fil_int.integratorInit(Ts);
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
    float exc = 0.0f;
    const float kp = 0.3f;

    Eigen::Matrix<float, 1, 4> K;
    // K << -1.7131f, -0.1553f, -0.0029f, 0.0023f * 2.0f;
    K << -2.1862f, -0.2010f, -0.0042f, 0.0100f;
    Eigen::Matrix<float, 4, 1> x_bar;
    x_bar.setZero();
    float M_mot;
    const float km = 36.9e-3;

    while (1) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        const float time = 1e-6f * (float)(duration_cast<microseconds>(m_Timer.elapsed_time()).count());
        // --------------------- THE LOOP ---------------------

        float w = myDataLogger.get_set_value(time); // get set values from the GUI

        // update sensor readings and readout values from io handler
        m_IO_handler->update();
        const float phi_fw_vel = m_IO_handler->get_phi_fw_vel();
        const float gz = m_IO_handler->get_gz();
        const float phi_bd = m_IO_handler->get_phi_bd();

        float i_des = 0.0f;
        const bool do_transition = m_IO_handler->get_and_reset_button_state();
        switch (m_state) {
            case INIT: {
                // ------------------- INIT -------------------
                // reset system
                m_fil_int.reset(0.0f);
                m_IO_handler->disable_escon();

                // switch to FLAT
                if (do_transition) {
                    m_state = FLAT;
                    m_IO_handler->enable_escon();
                }
                break;
            }
            case FLAT: {
                // ------------------- FLAT -------------------
                // simple P controller
                // i_des = kp * (w - phi_fw_vel + (exc + 2.0f * M_PIf));
                i_des = kp * (w - phi_fw_vel);

                // switch to BALANCE
                if (do_transition) {
                    m_state = BALANCE;
                    printf("switch to BALANCE\r\n");
                }
                break;
            }
            case BALANCE: {
                // ------------------- BALANCE ----------------
                // state space controller with integrator for velocity error
                x_bar << phi_bd, gz, phi_fw_vel, m_fil_int.applyConstrained(w - phi_fw_vel, -5.0f * km, 5.0f * km);
                M_mot = -K * x_bar;
                i_des = M_mot / km;

                // switch to INIT
                if (do_transition) {
                    m_state = INIT;
                    printf("switch to INIT\r\n");
                }
                break;
            }
            default:
                break;
        }

        // write current setpoint to motor
        i_des = saturate(i_des, -15.0f, 15.0f);
        m_IO_handler->write_current(i_des);

        myDataLogger.write_to_log(time,
                                  w,         // 1
                                  i_des,     // 2
                                  x_bar(0),  // 3
                                  x_bar(1),  // 4
                                  x_bar(2),  // 5
                                  x_bar(3)); // 6

        // GPA - do not overwrite exc if you want to excite via the GPA
        exc = myGPA.update(i_des, phi_fw_vel); // GPA calculates future excitation exc(k+1)
    }
}

float realtime_thread::saturate(float x, float ll, float ul)
{
    if (x > ul)
        return ul;
    else if (x < ll)
        return ll;
    return x;
}

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}

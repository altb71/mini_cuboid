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
    I_reg = PID_Cntrl(0, 1, 0, 0, Ts, -1000, 1000);
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
    float exc = 0.0f;
    float kp = 0.3f;

    Eigen::Matrix<float, 1, 4> K4;
    K4 << -1.7131f, -0.1553f, -0.0029f, 0.0023f * 2.0f;
    Eigen::Matrix<float, 4, 1> x_bar;
    float M_mot;
    float km = 36.9e-3;

    while (1) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        const float time = 1e-6f * (float)(duration_cast<microseconds>(m_Timer.elapsed_time()).count());
        // --------------------- THE LOOP ---------------------

        float w = myDataLogger.get_set_value(time); // get set values from the GUI

        m_IO_handler->update();
        const float phi_fw_vel = m_IO_handler->get_phi_fw_vel();
        const float phi_bd = m_IO_handler->get_phi_bd();
        const float gz = m_IO_handler->get_gz();

        float i_des = 0.0f;
        const bool do_transition = m_IO_handler->get_key_state();
        switch (m_state) {
            case INIT: {
                // ------------------- INIT -------------------

                I_reg.reset();
                m_IO_handler->disable_escon();

                // Switch to FLAT
                if (do_transition) {
                    m_state = FLAT;
                    m_IO_handler->enable_escon();
                }
                break;
            }
            case FLAT: {
                // ------------------- FLAT -------------------

                // i_des = kp * (w - phi_fw_vel + (exc + 2.0f * M_PIf));
                i_des = kp * (w - phi_fw_vel);

                // Switch to BALANCE
                if (do_transition) {
                    m_state = BALANCE;
                    printf("switch to BALANCE\r\n");
                }
                break;
            }
            case BALANCE: {
                // ------------------- BALANCE ----------------

                x_bar << phi_bd, gz, phi_fw_vel, I_reg(0 - phi_fw_vel);
                M_mot = -K4 * x_bar;
                i_des = M_mot / km;

                // Switch to INIT
                if (do_transition) {
                    m_state = INIT;
                    printf("switch to INIT\r\n");
                }
                break;
            }
            default:
                break;
        }
        m_IO_handler->write_current(i_des);

        myDataLogger.write_to_log(time,
                                  w,         // 1
                                  i_des,     // 2
                                  x_bar(0),  // 3
                                  x_bar(1),  // 4
                                  x_bar(2),  // 5
                                  x_bar(3)); // 6

        // GPA calculates future excitation exc(k+1)
        exc = myGPA.update(i_des, phi_fw_vel);

        // float i_des = 0.0f;
        // const bool do_transition = m_IO_handler->get_key_state();
        // switch (m_state) {
        //     case INIT: {
        //         I_reg.reset();
        //         m_IO_handler->disable_escon();
        //         if (do_transition) {
        //             m_state = FLAT;
        //             printf("switch to FLAT\r\n");
        //             m_IO_handler->enable_escon();
        //         }
        //         break;
        //     }
        //     case FLAT: {
        //         i_des = kp * (0 - m_IO_handler->get_phi_fw_vel());
        //         if (do_transition) {
        //             m_state = BALANCE;
        //             printf("switch to BALANCE\r\n");
        //         }
        //         break;
        //     }
        //     case BALANCE: {
        //         x_bar << m_IO_handler->get_phi_bd(), m_IO_handler->get_gz(), m_IO_handler->get_phi_fw_vel(),
        //             I_reg(0 - m_IO_handler->get_phi_fw_vel());
        //         M_mot = -K4 * x_bar;
        //         i_des = M_mot / km;
        //         if (do_transition) {
        //             m_state = INIT;
        //             printf("switch to INIT\r\n");
        //         }
        //         break;
        //     }
        //     default:
        //         break;
        // }

        // m_IO_handler->write_current(i_des);

        // myDataLogger.write_to_log(time,
        //                           u,
        //                           m_IO_handler->get_ax(),
        //                           m_IO_handler->get_ay(),
        //                           m_IO_handler->get_gz(),
        //                           m_IO_handler->get_phi_fw(),
        //                           m_IO_handler->get_phi_fw_vel());
    }
}

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}

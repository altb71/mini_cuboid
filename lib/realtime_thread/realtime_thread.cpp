#include "realtime_thread.h"

using namespace std;

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
    Eigen::Matrix<float, 1, 4> K4;
    K4 << -1.7131, -0.1553, -0.0029, 0.0023;
    Eigen::Matrix<float, 4, 1> x_bar;
    float M_mot;
    float km = 36.9e-3;
    float kp = 0.3;
    float time;

    while (1) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        time = 1e-6f * (float)(duration_cast<microseconds>(m_Timer.elapsed_time()).count());
        // --------------------- THE LOOP ---------------------

        m_IO_handler->update();

        float i_des = 0.0f;
        bool do_transition = false;
        if (m_IO_handler->get_key_state() && (time > 0.5f)) {
            do_transition = true;
            m_Timer.reset();
        }

        switch (m_state) {
            case INIT: {
                m_IO_handler->disable_escon();
                I_reg.reset();
                if (do_transition) {
                    m_state = FLAT;
                    printf("switch to FLAT\r\n");
                    m_IO_handler->enable_escon();
                }
                break;
            }
            case FLAT: {
                i_des = kp * (0 - m_IO_handler->get_phi_fw_vel());
                if (do_transition) {
                    m_state = BALANCE;
                    printf("switch to BALANCE\r\n");
                }
                break;
            }
            case BALANCE: {
                x_bar << m_IO_handler->get_phi_bd(), m_IO_handler->get_gz(), m_IO_handler->get_phi_fw_vel(),
                    I_reg(0 - m_IO_handler->get_phi_fw_vel());
                M_mot = -K4 * x_bar;
                i_des = M_mot / km;
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
    }
}

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}

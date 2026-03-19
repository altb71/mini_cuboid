#include "IO_handler.h"

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

// constructors
IO_handler::IO_handler(float Ts)
    : m_encoder(PA_8, PA_9, 4 * 2048)
    , m_a_out(PA_4)
    , m_d_out(PB_1)
    , m_button(PA_10)
    , m_spi(PA_12, PA_11, PA_1)
    , m_imu(m_spi, PB_0)
{
    m_button.rise(callback(this, &IO_handler::but_pressed)); // attach debounced key press function
    m_button_was_pressed = false;

    m_d_out = 0;
    m_encoder.reset(); // encoder reset

    m_imu.init_inav();
    m_imu.configuration();

    /* *** AUFGABEN *** :
    1.1, 1.2, 1.3    */
    m_lc_ax2ax = LinearCharacteristics(-16400.0f, 16580.0f, -9.81f, 9.81f);
    m_lc_ay2ay = LinearCharacteristics(-17120.0f, 15700.0f, -9.81f, 9.81f);
    m_lc_gz2gz = LinearCharacteristics(-32767.0f, 32768.0f, -1000.0f * M_PIf / 180.0f, 1000.0f * M_PIf / 180.0f);
    m_lc_i2u = LinearCharacteristics(-15.0f, 15.0f, 0.0f, 1.0f);

    /*  Aufgabe 3.1 Parametrieren  der Filter */
    float tau = 1.0f;
    m_fil_ax = IIR_filter(tau, Ts, 1.0f);
    m_fil_ay = IIR_filter(tau, Ts, 1.0f);
    m_fil_gz = IIR_filter(tau, Ts, tau);

    // differentiator filter
    m_fil_diff = IIR_filter(1.0f, Ts);
}

IO_handler::~IO_handler() {}

void IO_handler::update(void)
{
    m_phi_fw = m_encoder.getAngleRad();
    m_phi_fw_vel = m_fil_diff(m_phi_fw);

    m_ax = m_lc_ax2ax(m_imu.readAcc_raw(1));
    m_ay = m_lc_ay2ay(-m_imu.readAcc_raw(0));
    m_gz = m_lc_gz2gz(m_imu.readGyro_raw(2));

    m_phi_bd = -M_PIf / 4.0f + atan2f(m_fil_ax(m_ax), m_fil_ay(m_ay)) + m_fil_gz(m_gz);
}

float IO_handler::get_phi_fw(void) { return m_phi_fw; }

float IO_handler::get_phi_bd(void) { return m_phi_bd; }

float IO_handler::get_phi_fw_vel(void) { return m_phi_fw_vel; }

float IO_handler::get_ax(void) { return m_ax; }

float IO_handler::get_ay(void) { return m_ay; }

float IO_handler::get_gz(void) { return m_gz; }

void IO_handler::write_current(float i_des) { m_a_out = m_lc_i2u(i_des); }

void IO_handler::enable_escon(void) { m_d_out = 1; }

void IO_handler::disable_escon(void) { m_d_out = 0; }

bool IO_handler::get_key_state(void)
{
    bool button_was_pressed = m_button_was_pressed;
    m_button_was_pressed = false;
    return button_was_pressed;
}

void IO_handler::but_pressed() { m_button_was_pressed = true; }

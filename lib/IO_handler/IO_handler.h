#pragma once

#include <cstdint>

#include "DebounceIn.h"
#include "Encoder.h"
#include "IIRFilter.h"
#include "LinearCharacteristics.h"
#include "mpu6500_spi.h"

class IO_handler
{
public:
    IO_handler(float Ts);  // default constructor
    virtual ~IO_handler(); // deconstructor

    void update(void); // read both encoders and calculate speeds

    float get_phi_fw(void);     // get angle of motor k
    float get_phi_bd(void);     // get angle of motor k
    float get_phi_fw_vel(void); // get speed of motor k
    float get_ax(void);
    float get_ay(void);
    float get_gz(void);
    void write_current(float); // write current to motors (0,...) for motor 1, (1,...) for motor 2
    void enable_escon();
    void disable_escon();
    bool get_key_state(void);

private:
    Encoder m_encoder; // initialize encoder on PA_8 and PA_9
    AnalogOut m_a_out; // desired current values
    DigitalOut m_d_out;
    DebounceIn m_button;
    SPI m_spi; // mosi, miso, sclk
    mpu6500_spi m_imu;
    LinearCharacteristics m_lc_i2u;
    LinearCharacteristics m_lc_ax2ax, m_lc_ay2ay, m_lc_gz2gz; // map imu raw values to m/s^2 and rad/s
    // sensor states
    float m_phi_fw, m_phi_bd; // motor angle /rad
    float m_phi_fw_vel;       // motor speed / rad / s
    float m_ax, m_ay, m_gz;   // accelerations and gyroscope
    void but_pressed(void);
    bool m_button_was_pressed;
    float m_tau = 0.0f;
    IIRFilter m_fil_ax, m_fil_ay, m_fil_gz;
    IIRFilter m_fil_diff;
    /*  Aufgabe 3.1   */
};

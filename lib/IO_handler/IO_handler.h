#pragma once
/* class IO_handler
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>

#include "Encoder.h"
#include "IIR_filter.h"
#include "LinearCharacteristics.h"
#include "mpu6500_spi.h"

class IO_handler
{
public:
    IO_handler(float Ts);               // default constructor
    virtual ~IO_handler();              // deconstructor
    void read_sensors_calc_speed(void); // read both encoders and calculate speeds
    float get_phi_fw(void);             // get angle of motor k
    float get_phi_bd(void);             // get angle of motor k
    float get_vphi_fw(void);            // get speed of motor k
    float get_ax(void);
    float get_ay(void);
    float get_gz(void);
    void write_current(float); // write current to motors (0,...) for motor 1, (1,...) for motor 2
    void enable_escon();
    void disable_escon();
    bool get_key_state(void);

private:
    ///------------- Encoder -----------------------
    Encoder counter; // initialize counter on PA_8 and PA_9
    AnalogOut i_des;        // desired current values
    DigitalOut i_enable;
    InterruptIn button;
    //-------------------------------------
    SPI spi; // mosi, miso, sclk
    mpu6500_spi imu;
    LinearCharacteristics i2u;
    LinearCharacteristics ax2ax, ay2ay, gz2gz; // map imu raw values to m/s^2 and rad/s
    Timer t_but; // define button time        //
    // sensor states
    float phi_fw, phi_bd;   // motor angle /rad
    float Vphi_fw;          // motor speed / rad / s
    float accx, accy, gyrz; // accelerations and gyroscope
    void but_pressed(void);
    void but_released(void);
    bool key_was_pressed;
    IIR_filter fil_ax, fil_ay, fil_gz;
    IIR_filter diff;
    /*  Aufgabe 3.1   */
};

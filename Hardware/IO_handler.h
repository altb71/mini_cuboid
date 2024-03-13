#pragma once
/* class IO_handler
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "LinearCharacteristics.h"
#include "Enc_unwrap_scale.h"
#include "mpu6500_spi.h"
#include "IIR_filter.h"


class IO_handler
{
public:
    IO_handler(float Ts);        // default constructor
    virtual ~IO_handler();   // deconstructor
    void read_sensors_calc_speed(void);       // read both encoders and calculate speeds
    float get_phi_fw(void);         // get angle of motor k
    float get_phi_bd(void);         // get angle of motor k
    float get_vphi_fw(void);          // get speed of motor k
    float get_ax(void);
    float get_ay(void);
    float get_gz(void);
    void write_current(float);  // write current to motors (0,...) for motor 1, (1,...) for motor 2
    void enable_escon();
    void disable_escon();
    bool get_key_state(void);
   
private:
    ///------------- Encoder -----------------------
    EncoderCounter counter;    // initialize counter on PA_6 and PC_7
    AnalogOut i_des;           // desired current values
    DigitalOut i_enable;
    InterruptIn button;
    mpu6500_spi imu;
    //-------------------------------------
    SPI spi;                    // mosi, miso, sclk
    LinearCharacteristics i2u;
    LinearCharacteristics ax2ax,ay2ay,gz2gz;    // map imu raw values to m/s^2 and rad/s
    IIR_filter fil_ax,fil_ay,fil_gz;
    Enc_unwrap_scale uw;
    Timer t_but;                            // define button time        // 
    // sensor states
    float phi_fw,phi_bd;          // motor angle /rad
    float Vphi_fw;           // motor speed / rad / s
    float accx,accy,gyrz;       // accelerations and gyroscope
    void but_pressed(void);
    void but_released(void);
    bool key_was_pressed;
    /*  Aufgabe 3.1   */ 

};
#pragma once
/* class sensors_actuators
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


class sensors_actuators
{
public:
    sensors_actuators(float Ts);        // default constructor
    virtual ~sensors_actuators();   // deconstructor
    void read_sensors_calc_speed(void);       // read both encoders and calculate speeds
    float get_phi_fw(void);         // get angle of motor k
    float get_phi_bd(void);         // get angle of motor k
    float get_the_bd(void);         // get angle of motor k
    float get_om_fw(void);          // get speed of motor k
    float get_ax(void);
    float get_ay(void);
    float get_az(void);
    float get_gx(void);
    float get_gy(void);
    float get_gz(void);
    float get_curr_setvalue(void);
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
    IIR_filter fil_accx,fil_accy,fil_accz,fil_gyrx,fil_gyry;
    IIR_filter diff;        // DIfferentiator to get speed of fw
    LinearCharacteristics ax2ax,ay2ay,az2az,gx2gx,gy2gy,gz2gz;    // map imu raw values to m/s^2 and rad/s
    Enc_unwrap_scale uw;
    Timer t_but;                            // define button time        // 
    // sensor states
    float phi_fw,phi_bd,the_bd;          // motor angle /rad
    float om_fw;                // motor speed / rad / s
    float accx,accy,accz,gyrx,gyry,gyrz,accx_sens,accy_sens,gyrx_sens,gyry_sens;       // accelerations and gyroscope
    void est_angle(void);
    void but_pressed(void);
    void but_released(void);
    bool key_was_pressed;
    float curr_setvalue;
    float sq2 = 0.70710678;

};
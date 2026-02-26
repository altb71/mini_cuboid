#pragma once
/* class sensors_actuators
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "IIR_filter.h"
#include "LinearCharacteristics.h"
#include "Enc_unwrap_scale.h"
#include "unwrap_2pi.h"
#include "mpu6500_spi.h"
#include "minicube_parametermap.h"
#define MAX_FW_SPEED 420


class sensors_actuators
{
public:
    sensors_actuators(float Ts);        // default constructor
    virtual ~sensors_actuators();   // deconstructor
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
    float ax_fil,ay_fil;
    void force_curr(float);
    bool get_but(void);
private:
    IIR_filter di;
    IIR_filter fil_ax,fil_ay,fil_gz, dif_ax, dif_ay;

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
    Enc_unwrap_scale uw;
    unwrap_2pi uw2pi;
    Timer t_but;                            // define button time        // 
    // sensor states
    float phi_fw;          // motor angle /rad
    float phi_bd;           // body angle / rad
    float Vphi_fw;           // motor speed / rad / s
    float accx,accy,gyrz,gyrz_fil;       // accelerations and gyroscope
    void but_pressed(void);
    void but_released(void);
    bool global_enable;
    bool key_was_pressed;
    
};
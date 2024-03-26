#include "IO_handler.h"

#define PI 3.1415927
// constructors

IO_handler::IO_handler(float Ts) : counter(PA_8, PA_9),
                            i_enable(PB_1),button(PA_10),i_des(PA_4),uw(4*2048,16),spi(PA_12, PA_11, PA_1),imu(spi, PB_0)
{
    button.fall(callback(this, &IO_handler::but_pressed));          // attach key pressed function
    button.rise(callback(this, &IO_handler::but_released));         // attach key pressed function
    key_was_pressed = false;
    i_enable = 0;
    counter.reset();   // encoder reset
    imu.init_inav();
    imu.configuration();
    /* *** AUFGABEN *** :
    1.1, 1.2, 1.3    */
    ax2ax = LinearCharacteristics(-16270,16580,-9.81,9.81);
    ay2ay = LinearCharacteristics(-17200,15650,-9.81,9.81);
    gz2gz = LinearCharacteristics(-32767,32768,-1000*PI/180,1000*PI/180);
    i2u = LinearCharacteristics(-15,15,0,1);
    /*  Aufgabe 3.1 Parametrieren  der Filter */
    float tau = 1;
    fil_ax = IIR_filter(tau,Ts,1);
    fil_ay = IIR_filter(tau,Ts,1);
    fil_gz = IIR_filter(tau,Ts,tau);
    // Initialisiere Ableitungfilter
    diff = IIR_filter(1,Ts);

}
// Deconstructor
IO_handler::~IO_handler() {} 

void IO_handler::read_sensors_calc_speed(void)
{
    phi_fw = uw(counter);
    Vphi_fw = diff(phi_fw);
    //-------------- read imu ------------
    accx = ax2ax(imu.readAcc_raw(1));
    accy = ay2ay(-imu.readAcc_raw(0));
    gyrz = gz2gz(imu.readGyro_raw(2));
    // Komplementaerfilter
    phi_bd = atan2(fil_ax(accx),fil_ay(accy))+fil_gz(gyrz) - PI/4;

}

void IO_handler::enable_escon(void)
{
    i_enable = 1;    
}
void IO_handler::disable_escon(void)
{
    i_enable = 0;    
}

void IO_handler::write_current(float _i_des)
{
        i_des = i2u(_i_des);   
}

float IO_handler::get_phi_bd(void)
{
    return phi_bd;
}
float IO_handler::get_phi_fw(void)
{
    return phi_fw;
}
float IO_handler::get_vphi_fw(void)
{
    return Vphi_fw;
}
float IO_handler::get_ax(void)
{
    return accx;
}
float IO_handler::get_ay(void)
{
    return accy;
}
float IO_handler::get_gz(void)
{
    return gyrz;
}
// start timer as soon as Button is pressed
void IO_handler::but_pressed()
{
    t_but.start();
    key_was_pressed = false;
}
 
// evaluating statemachine
void IO_handler::but_released()
{
     // readout, stop and reset timer
    float ButtonTime = t_but.read();
    t_but.stop();
    t_but.reset();
    if(ButtonTime > 0.05f && ButtonTime < 0.5) 
        key_was_pressed = true;
}
bool IO_handler::get_key_state(void)
{
    bool temp = key_was_pressed;
    key_was_pressed = false;
    return temp;
} 
#include "sensors_actuators.h"

#define PI 3.1415927
// constructors

sensors_actuators::sensors_actuators(float Ts) : counter(PA_8, PA_9),
                            i_enable(PB_1),button(PA_10),i_des(PA_4),uw(4*2048,16),spi(PA_12, PA_11, PA_1),imu(spi, PB_0)
{
    button.fall(callback(this, &sensors_actuators::but_pressed));          // attach key pressed function
    button.rise(callback(this, &sensors_actuators::but_released));         // attach key pressed function
    key_was_pressed = false;
    i_enable = 0;
    counter.reset();   // encoder reset
    imu.init_inav();
    imu.configuration();
    ax2ax = LinearCharacteristics(-80  ,16650,0,9.81);             // parametrize object based on sensor values from calibration
    ay2ay = LinearCharacteristics(-1300,15000,0,9.81);
    az2az = LinearCharacteristics(-200 ,15600,0,9.81);
    //ax2ax = LinearCharacteristics(1,0);             // parametrize object based on sensor values from calibration
    //ay2ay = LinearCharacteristics(1,0);
    //az2az = LinearCharacteristics(1,0);
    gx2gx = LinearCharacteristics(-32767,32768,-1000*PI/180,1000*PI/180);
    gy2gy = LinearCharacteristics(-32767,32768,-1000*PI/180,1000*PI/180);
    gz2gz = LinearCharacteristics(-32767,32768,-1000*PI/180,1000*PI/180);
    i2u = LinearCharacteristics(-15,15,0,1.0f);
    float tau = 1.5;
    fil_accx = IIR_filter(tau,Ts,1);
    fil_accy = IIR_filter(tau,Ts,1);
    fil_accz = IIR_filter(tau,Ts,1);
    fil_gyrx = IIR_filter(tau,Ts,tau);
    fil_gyry = IIR_filter(tau,Ts,tau);
    diff = IIR_filter(3*Ts,Ts);
}
// Deconstructor
sensors_actuators::~sensors_actuators() {} 

void sensors_actuators::read_sensors_calc_speed(void)
{
    phi_fw = uw(counter);
    om_fw = diff(phi_fw);//
    //-------------- read imu ------------
    accx_sens = ax2ax(imu.readAcc_raw(1));
    accy_sens = ay2ay(-imu.readAcc_raw(2));
    accz = az2az(-imu.readAcc_raw(0));
    gyrx_sens = gx2gx(imu.readGyro_raw(1));
    gyry_sens = gy2gy(-imu.readGyro_raw(2));
    gyrz = gz2gz(-imu.readGyro_raw(0));
    // calculate sensor  data in bd coordinates
    accx =  sq2 * accx_sens + sq2 * accy_sens;
    accy = -sq2 * accx_sens + sq2 * accy_sens;
    gyrx =  sq2 * gyrx_sens + sq2 * gyry_sens;
    gyry = -sq2 * gyrx_sens + sq2 * gyry_sens;
    est_angle();            // complementary filter
}

/* est_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students. 
*/
void sensors_actuators::est_angle(void)
{
    //phi_bd = atan2(fil_accy(accy),fil_accx(accx)) + fil_gyrz(gyrz) -PI/4;
    float fax, fay,faz;
    fax = fil_accx(accx);
    fay = fil_accy(accy);
    faz = fil_accz(accz);
    phi_bd = atan2(fay,faz) + fil_gyrx(gyrx) -0; 
    the_bd = atan2(-fax,faz) + fil_gyry(gyry) - 0;
    
}


void sensors_actuators::enable_escon(void)
{
    i_enable = 1;    
}
void sensors_actuators::disable_escon(void)
{
    i_enable = 0;    
}

void sensors_actuators::write_current(float _i_des)
{
        i_des = i2u(_i_des);
        curr_setvalue = _i_des; 
}

float sensors_actuators::get_phi_bd(void)
{
    return phi_bd;
}
float sensors_actuators::get_the_bd(void)
{
    return the_bd;
}
float sensors_actuators::get_phi_fw(void)
{
    return phi_fw;
}
float sensors_actuators::get_om_fw(void)
{
    return om_fw;
}
// ------------------------------------------------
float sensors_actuators::get_ax(void)
{
    return accx;
}
float sensors_actuators::get_ay(void)
{
    return accy;
}
float sensors_actuators::get_az(void)
{
    return accz;
}
// ------------------------------------------------
float sensors_actuators::get_gx(void)
{
    return gyrx;
}
float sensors_actuators::get_gy(void)
{
    return gyry;
}
float sensors_actuators::get_gz(void)
{
    return gyrz;
}
float sensors_actuators::get_curr_setvalue(void)
{
    return curr_setvalue;
}
// start timer as soon as Button is pressed
void sensors_actuators::but_pressed()
{
    t_but.start();
    key_was_pressed = false;
}
 
// evaluating statemachine
void sensors_actuators::but_released()
{
     // readout, stop and reset timer
    float ButtonTime = t_but.read();
    t_but.stop();
    t_but.reset();
    if(ButtonTime > 0.05f && ButtonTime < 0.5) 
        key_was_pressed = true;
}
bool sensors_actuators::get_key_state(void)
{
    bool temp = key_was_pressed;
    key_was_pressed = false;
    return temp;
} 
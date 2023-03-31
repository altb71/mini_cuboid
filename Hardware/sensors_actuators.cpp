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
    ax2ax = LinearCharacteristics(-16380,16480,-9.81,9.81);             // parametrize object based on sensor values from calibration
    ay2ay = LinearCharacteristics(-16950,15830,-9.81,9.81);
    gz2gz = LinearCharacteristics(-32767,32768,-1000*PI/180,1000*PI/180);
    i2u = LinearCharacteristics(-15,15,0,1.0f);
    float tau = 1.0;
    fil_accx = IIR_filter(tau,Ts,1);
    fil_accy = IIR_filter(tau,Ts,1);
    fil_gyrz = IIR_filter(tau,Ts,tau);
    diff = IIR_filter(Ts);
}
// Deconstructor
sensors_actuators::~sensors_actuators() {} 

void sensors_actuators::read_sensors_calc_speed(void)
{
    phi_fw = uw(counter);
    om_fw = diff(phi_fw);//
    //-------------- read imu ------------
    accx = ax2ax(imu.readAcc_raw(1));
    accy = ay2ay(-imu.readAcc_raw(0));
    gyrz = gz2gz(imu.readGyro_raw(2));
    est_angle();            // complementary filter
}

/* est_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students. 
*/
void sensors_actuators::est_angle(void)
{
    //phi_bd = atan2(fil_accy(accy),fil_accx(accx)) + fil_gyrz(gyrz) -PI/4;
    phi_bd = atan2(fil_accx(accx),fil_accy(accy)) + fil_gyrz(gyrz) -PI/4;

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
}

float sensors_actuators::get_phi_bd(void)
{
    return phi_bd;
}
float sensors_actuators::get_phi_fw(void)
{
    return phi_fw;
}
float sensors_actuators::get_om_fw(void)
{
    return om_fw;
}
float sensors_actuators::get_ax(void)
{
    return accx;
}
float sensors_actuators::get_ay(void)
{
    return accy;
}
float sensors_actuators::get_gz(void)
{
    return gyrz;
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
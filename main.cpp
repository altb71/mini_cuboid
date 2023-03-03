#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "state_machine.h"
#include "IIR_filter.h"

#define WAIT_MS(x) ThisThread::sleep_for(chrono::milliseconds(x));

static BufferedSerial serial_port(USBTX, USBRX,115200);

/* 
This is the main function of embedded project "mini_cuboid" ZHAW FS23
Altenburger February 2023
*/

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- mini cuboid,
    float Ts = 0.002f;                      // sampling time, typically approx 1/500
    sensors_actuators hardware(Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&hardware,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&loop,0.02);
    WAIT_MS(200);
    printf("- - - - MiniCuboid Start! - - - \r\n");
// ----------------------------------
    loop.start_loop();
    WAIT_MS(20);
    sm.start_loop();
    IIR_filter lp(0.1,Ts,1);        // short hack, to test filter
    for(uint16_t k=0;k<100;k++)
        printf("%f ",lp(1));        // do a step response

    while(1)
        {
        WAIT_MS(500);
        printf("ax: %f ay: %f gz: %f\r\n",hardware.get_ax(),hardware.get_ay(),hardware.get_gz());
        }
}   // END OF main


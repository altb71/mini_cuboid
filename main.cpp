#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "realtime_thread.h"
#include "IO_handler.h"
#include "state_machine.h"

#define WAIT_MS(x) ThisThread::sleep_for(chrono::milliseconds(x));

static BufferedSerial serial_port(USBTX, USBRX,115200);

/* 
This is the main function of embedded project "mini_cuboid" ZHAW FS23
Altenburger February 2023
*/

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

#include "IIR_filter.h"

int main()
{

    // --------- mini cuboid,
    float Ts = 0.002f;                      // sampling time, typically approx 1/500
    IO_handler hardware(Ts);         // in this class all the physical ios are handled
    realtime_thread rt_thread(&hardware,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&rt_thread,0.02);
    WAIT_MS(200);
    printf("- - - - MiniCuboid Start! - - - \r\n");
// ----------------------------------
    rt_thread.start_loop();
    WAIT_MS(20);
    sm.start_loop();
    IIR_filter TP(.1,Ts,1);
    float ti = 0;
    while(1)
        {
        WAIT_MS(5);
        //printf("ax: %f ay: %f gz: %f\r\n",hardware.get_ax(),hardware.get_ay(),hardware.get_gz());
        // Aufgabe 2.4
        /*if(ti<.25)
            printf("%f %f\r\n",ti,TP(1));
        ti += Ts;*/
        
        }
}   // END OF main


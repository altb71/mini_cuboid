#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "state_machine.h"
#include "GPA.h"
#include "minicube_parametermap.h"
/*
software (running) for "Nacht der Technik" July 2022. Concept
    - 2 types of controllers running in ControllerLoop.cpp at 500Hz
        * balance controller at unstable position on edge
        * PI-controller to keep disc speed at 0, when lying on edge
    - controllers are enabled/disabled in state_machine.cpp
    - main things are performed in state_machine.cpp
    - each cuboid has the same software!
    - individual cuboids are identified by their ID number (see lines below in main.cpp)
    - choreography is read in choreography.h
    - choreography.h is created with Matlab: generateChoreography.m
*/



// Mini-cuboid for lab, see Matlab-code at end of this file
static BufferedSerial serial_port(USBTX, USBRX);
float Ts = 0.002f;    // sampling time, typically approx 1/500
GPA          myGPA( .7,  250,    30,4,4, Ts); // para for plant identification (currently not used)
//******************************************************************************
//---------- main loop -------------
//******************************************************************************
int main()
{
    sensors_actuators hardware(Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&hardware,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&loop,0.02);
    ThisThread::sleep_for(200);
    uint32_t *uid = (uint32_t *)0x1FFF7590;
//    printf("\r\nUnique ID: %08X %08X %08X \r\n", uid[0], uid[1], uid[2]);
    printf("\r\nUnique ID: %08X %08X \r\n", uid[1], uid[0]);
// ----------------------------------
    ThisThread::sleep_for(20);
    loop.start_loop();
    ThisThread::sleep_for(20);
    sm.start_loop();
    while(1)
        ThisThread::sleep_for(200); 
}   // END OF main
/*      MATLAB CODE for controller design
m = 0.816;
J_geh=7.66E-4;
J_rot = 2.81E-4;
R = 0.066;
km = 36.9E-3;
g=9.81;
J = J_geh + m*R^2;
%% Zustandsregler
A=[0 1;m*g*R/J 0];
B=[0;-1/J];
K=place(A,B,10*[-1+1j -1-1j])
%% Erweiterung auf System 3ter Ordnung
A3_ = [A zeros(2,1);zeros(1,3)];
B3_ = [B;1/J_rot];
C3_ = [1 0 0;0 1 0;0 -1 1];
s3_ = ss(A3_,B3_,C3_,0);
s3 = ss2ss(s3_,C3_);
%% Erweiterung auf 4ter Ordnung
A4 = [s3.a zeros(3,1);-[0 0 1] 0];
B4 = [s3.b;0];
K4 = place(A4,B4,10*[-1+1j -1-1j -1.5 -.1])
%% Scheibendrehzahl regeln
G_sb=tf(1/km*6.26E7,[1 2250 500000 0]);
Tn = .02;
kp=db2mag(-30);
PI=tf([Tn 1],[Tn 0]);
bode(kp*PI*G_sb);grid on
pid(kp*PI)
*/
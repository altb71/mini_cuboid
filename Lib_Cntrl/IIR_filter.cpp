#include "IIR_filter.h"


// constructors
IIR_filter::IIR_filter(float Ts)  // Ableitungsfilter
{
    a0 = -1;
    b0 = 0;
    b1 = Ts;
    yk = 0;
    u_km1 = 0;
}
IIR_filter::IIR_filter(float tau,float Ts)  // Ableitungsfilter
{
    a0 = 0;
    b0 = -1/Ts;
    b1 = 1/Ts;
    yk = 0;
    u_km1 = 0;
}
IIR_filter::IIR_filter(float tau,float Ts,float K)  // Konstruktor fuer TP-filter
{
/* *** AUFGABEN *** :
    2.1, 2.2, 2.3    */
    b0 = K*Ts/tau;
    a0 = -(1-Ts/tau);
    b1 = 0;
    yk = 0;
    u_km1 = 0;

}

// Methods:
float IIR_filter::eval(float u)
{
/* *** AUFGABEN *** :
    2.3              */
    float y_new = -a0*yk + b1 * u + b0* u_km1;
    u_km1 = u;
    yk = y_new;
    return y_new;       // this has to be modified!!!
}


// Deconstructor
IIR_filter::~IIR_filter() {} 
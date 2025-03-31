#include "IIR_filter.h"


// constructors
IIR_filter::IIR_filter(float tau,float Ts)
{
    b1 = 1/Ts;
    b0 = -1/Ts;
    a0 = 0;
    yk = 0;
    uk = 0;

}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{
/* *** AUFGABEN *** :
    2.1, 2.2, 2.3    */
    b0 = K * Ts/tau;
    b1 = 0;
    a0 = Ts/tau-1;
    yk = 0;
    uk = 0;

}

// Methods:

float IIR_filter::eval(float u)
{
/* *** AUFGABEN *** :
    2.3              */
    float y = b1 * u + b0 * uk -a0 * yk;
    yk = y;         // shift yk for next step
    uk = u;
    return y;       // this has to be modified!!!
}


// Deconstructor
IIR_filter::~IIR_filter() {} 
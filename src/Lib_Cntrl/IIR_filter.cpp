#include "IIR_filter.h"


// constructors
IIR_filter::IIR_filter(float tau,float Ts)
{


}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{
/* *** AUFGABEN *** :
    2.1, 2.2, 2.3    */
    b0 = K * Ts/tau;
    a0 = Ts/tau-1;
    yk = 0;

}

// Methods:

float IIR_filter::eval(float u)
{
/* *** AUFGABEN *** :
    2.3              */
    float y = b0 * u -a0 * yk;
    yk = y;         // shift yk for next step

    return y;       // this has to be modified!!!
}


// Deconstructor
IIR_filter::~IIR_filter() {} 
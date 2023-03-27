#include "IIR_filter.h"

// constructors
IIR_filter::IIR_filter(float tau,float Ts)
{
}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{
    this->Ts = Ts;
    this->a0 = -tau/(tau+Ts);
    this->b0 = K * Ts/(tau+Ts);
    this->y_old = 0.0;  
}

// Methods:
float IIR_filter::eval(float u)
{
    float y;
    y = b0 * u -a0 * y_old;
    y_old = y;
    return y;       // this has to be modified!!!
}


// Deconstructor
IIR_filter::~IIR_filter() {} 
#include "IIR_filter.h"

// constructors
IIR_filter::IIR_filter(float Ts)
{
    this->Ts = Ts;
    this->a0 = 0;
    this->b0 = -1/Ts;
    this->b1 = 1/Ts;
    this->u_old = 0.0;
    this->y_old = 0.0;
}
IIR_filter::IIR_filter(float tau,float Ts)
{
}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{
    this->Ts = Ts;
    this->a0 = -tau/(tau+Ts);
    this->b0 = K * Ts/(tau+Ts);
    this->b1 = 0;
    this->u_old = 0.0;
    this->y_old = 0.0;  
}

// Methods:
float IIR_filter::eval(float u)
{
    float y;
    y = b1 * u +b0 * u_old  -a0 * y_old;
    u_old = u;
    y_old = y;
    return y;       // this has to be modified!!!
}


// Deconstructor
IIR_filter::~IIR_filter() {} 
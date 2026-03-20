#include "IIR_Filter.h"

#include <math.h>

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f // pi
#endif

IIR_Filter::~IIR_Filter() {}

// First Order Lowpass Filter
// Time continuous prototype: G(s) = wcut / (s + wcut)
// Discretization method: Tustin

void IIR_Filter::lowPass1Init(float fcut, float Ts)
{
    // a0 = pi*Ts*fcut + 1;
    // b0 = pi*Ts*fcut;
    // b1 = b0;
    // a1 = pi*Ts*fcut - 1;
    const float a0 = M_PIf * Ts * fcut + 1.0f;
    const float b0 = M_PIf * Ts * fcut;
    const float b1 = b0;
    const float a1 = M_PIf * Ts * fcut - 1.0f;
    m_b0 = b0 / a0;
    m_b1 = b1 / a0;
    m_a1 = a1 / a0;

    reset(0.0f, 0.0f);
}

// First Order Differentiating Lowpass Filter
// Time continuous prototype: G(s) = wcut * s / (s + wcut)
// Discretization method: Tustin

void IIR_Filter::differentiatingLowPass1Init(float fcut, float Ts)
{
    // a0 = pi*Ts*fcut + 1;
    // b0 = 2*pi*fcut;
    // b1 = -b0;
    // a1 = pi*Ts*fcut - 1;
    const float a0 = M_PIf * Ts * fcut + 1.0f;
    const float b0 = 2.0f * M_PIf * fcut;
    const float b1 = -b0;
    const float a1 = M_PIf * Ts * fcut - 1.0f;
    m_b0 = b0 / a0;
    m_b1 = b1 / a0;
    m_a1 = a1 / a0;

    reset(0.0f, 0.0f);
}

float IIR_Filter::evaluate(const float u_k)
{
    // update filter output
    const float y_k = m_b0 * u_k + m_b1 * m_u_kmin1 - m_a1 * m_y_kmin1;

    // store values for next iteration
    m_u_kmin1 = u_k;
    m_y_kmin1 = y_k;

    return y_k;
}

void IIR_Filter::reset(float u_kmin1, float y_kmin1)
{
    m_u_kmin1 = u_kmin1;
    m_y_kmin1 = y_kmin1;
}

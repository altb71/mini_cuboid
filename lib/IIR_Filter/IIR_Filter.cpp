#include "IIR_Filter.h"

#include <math.h>

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f // pi
#endif

// Integrator
// Time continuous prototype: G(s) = 1 / s
// Discretization method: Euler

void IIR_Filter::integratorInit(const float Ts)
{
    filter.order = 1;
    integratorUpdate(Ts);
    reset(0.0f);
}

void IIR_Filter::integratorUpdate(const float Ts)
{
    filter.B[0] = Ts;
    filter.B[1] = 0.0f;
    filter.B[2] = 0.0f;
    filter.A[0] = -1.0f;
    filter.A[1] = 0.0f;
}

// Differentiator
// Time continuous prototype: G(s) = s
// Discretization method: Euler

void IIR_Filter::differentiatorInit(const float Ts)
{
    filter.order = 1;
    differentiatorUpdate(Ts);
    resetDifferentingFilterToZero(0.0f);
}

void IIR_Filter::differentiatorUpdate(const float Ts)
{
    filter.B[0] = 1.0f / Ts;
    filter.B[1] = -1.0f / Ts;
    filter.B[2] = 0.0f;
    filter.A[0] = 0.0f;
    filter.A[1] = 0.0f;
}

// First Order Lowpass Filter
// Time continuous prototype: G(s) = wcut / (s + wcut)
// Discretization method: Tustin

void IIR_Filter::lowPass1Init(const float fcut, const float Ts)
{
    filter.order = 1;
    lowPass1Update(fcut, Ts);
    reset(0.0f);
}

void IIR_Filter::lowPass1Update(const float fcut, const float Ts)
{
    // a0 = pi*Ts*fcut + 1;
    // b0 = pi*Ts*fcut;
    // b1 = b0;
    // a1 = pi*Ts*fcut - 1;
    const float a0 = M_PIf * Ts * fcut + 1.0f;
    const float b0 = M_PIf * Ts * fcut;
    const float b1 = b0;
    const float a1 = M_PIf * Ts * fcut - 1.0f;
    filter.A[1] = 0.0f;
    filter.B[0] = b0 / a0;
    filter.B[1] = b1 / a0;
    filter.B[2] = 0.0f;
    filter.A[0] = a1 / a0;
}

// First Order Differentiating Lowpass Filter
// Time continuous prototype: G(s) = wcut * s / (s + wcut)
// Discretization method: Tustin

void IIR_Filter::differentiatingLowPass1Init(const float fcut, const float Ts)
{
    filter.order = 1;
    differentiatingLowPass1Update(fcut, Ts);
    resetDifferentingFilterToZero(0.0f);
}

void IIR_Filter::differentiatingLowPass1Update(const float fcut, const float Ts)
{
    // a0 = pi*Ts*fcut + 1;
    // b0 = 2*pi*fcut;
    // b1 = -b0;
    // a1 = pi*Ts*fcut - 1;
    const float a0 = M_PIf * Ts * fcut + 1.0f;
    const float b0 = 2.0f * M_PIf * fcut;
    const float b1 = -b0;
    const float a1 = M_PIf * Ts * fcut - 1.0f;
    filter.A[1] = 0.0f;
    filter.B[0] = b0 / a0;
    filter.B[1] = b1 / a0;
    filter.B[2] = 0.0f;
    filter.A[0] = a1 / a0;
}

// First Order Lead or Lag Filter
// Time continuous prototype: G(s) = (wPole / wZero) * (s + wZero) / (s + wPole)
// Discretization method: Tustin with prewarping

void IIR_Filter::leadLag1Init(const float fZero, const float fPole, const float Ts)
{
    filter.order = 1;
    leadLag1Update(fZero, fPole, Ts);
    reset(0.0f);
}

void IIR_Filter::leadLag1Update(const float fZero, const float fPole, const float Ts)
{
    const float wZero = (2.0f / Ts) * tanf(M_PIf * fZero * Ts);
    const float wPole = (2.0f / Ts) * tanf(M_PIf * fPole * Ts);
    const float k = 1.0f / (Ts * wPole + 2.0f);

    filter.B[0] = wPole * (Ts * wZero + 2.0f) / wZero * k;
    filter.B[1] = wPole * (Ts * wZero - 2.0f) / wZero * k;
    filter.B[2] = 0.0f;
    filter.A[0] = (Ts * wPole - 2.0f) * k;
    filter.A[1] = 0.0f;
}

void IIR_Filter::phaseComp1Init(const float fCenter, const float phaseLift, const float Ts)
{
    filter.order = 1;
    phaseComp1Update(fCenter, phaseLift, Ts);
    reset(0.0f);
}

void IIR_Filter::phaseComp1Update(const float fCenter, const float phaseLift, const float Ts)
{
    const float sn = sinf(M_PIf / 180.0f * phaseLift);
    const float k = sqrtf((1.0f - sn) / (1.0f + sn));
    const float fZero = fCenter * k;
    const float fPole = fCenter / k;

    leadLag1Update(fZero, fPole, Ts);
}

// Second Order Notch Filter
// Time continuous prototype: G(s) = (s^2 + wcut^2) / (s^2 + 2 * D * wcut * s + wcut^2)
// Discretization method: Tustin with prewarping

void IIR_Filter::notchInit(const float fcut, const float D, const float Ts)
{
    filter.order = 2;
    notchUpdate(fcut, D, Ts);
    reset(0.0f);
}

void IIR_Filter::notchUpdate(const float fcut, const float D, const float Ts)
{
    // prewarp is done implicitly
    const float omega = 2.0f * M_PIf * fcut * Ts;
    const float sn = sinf(omega);
    const float cs = cosf(omega);

    filter.B[0] = 1.0f / (1.0f + D * sn);
    filter.B[1] = -2.0f * cs * filter.B[0];
    filter.B[2] = filter.B[0];
    filter.A[0] = filter.B[1];
    filter.A[1] = (1.0f - D * sn) * filter.B[0];
}

// Second Order Lowpass Filter
// Time continuous prototype: G(s) = wcut^2 / (s^2 + 2 * D * wcut * s + wcut^2)
// Discretization method: Euler

void IIR_Filter::lowPass2Init(const float fcut, const float D, const float Ts)
{
    filter.order = 2;
    lowPass2Update(fcut, D, Ts);
    reset(0.0f);
}

void IIR_Filter::lowPass2Update(const float fcut, const float D, const float Ts)
{
    const float wcut = 2.0f * M_PIf * fcut;
    const float k1 = 2.0f * D * Ts * wcut;

    filter.A[1] = 1.0f / (Ts * Ts * wcut * wcut + k1 + 1.0f);
    filter.B[0] = 1.0f - filter.A[1] * (1.0f + k1);
    filter.B[1] = 0.0f;
    filter.B[2] = 0.0f;
    filter.A[0] = filter.B[0] - 1.0f - filter.A[1];
}

// Second Order Lead or Lag Filter
// Time continuous prototype:
// G(s) = (wPole^2 / wZero^2) * (s^2 + 2*DZero*wZero*s + wZero^2) / (s^2 + 2*DPole*wPole*s + wPole^2)
// Discretization method: Tustin with prewarping

void IIR_Filter::leadLag2Init(const float fZero, const float DZero, const float fPole, const float DPole, const float Ts)
{
    filter.order = 2;
    leadLag2Update(fZero, DZero, fPole, DPole, Ts);
    reset(0.0f);
}

void IIR_Filter::leadLag2Update(const float fZero, const float DZero, const float fPole, const float DPole, const float Ts)
{
    // prewarp is done implicitly
    const float omegaZero = 2.0f * M_PIf * fZero * Ts;
    const float snZero = sinf(omegaZero);
    const float csZero = cosf(omegaZero);
    const float omegaPole = 2.0f * M_PIf * fPole * Ts;
    const float snPole = sinf(omegaPole);
    const float csPole = cosf(omegaPole);
    const float k0 = 1.0f / (1.0f + DPole * snPole);
    const float k1 = k0 * (csPole - 1.0f) / (csZero - 1.0f);

    filter.B[0] = (1.0f + DZero * snZero) * k1;
    filter.B[1] = -2.0f * csZero * k1;
    filter.B[2] = (1.0f - DZero * snZero) * k1;
    filter.A[0] = -2.0f * csPole * k0;
    filter.A[1] = (1.0f - DPole * snPole) * k0;
    // filter.A[1] = filter.B[0] + filter.B[1] + filter.B[2] - 1.0f - filter.A[0];
}

void IIR_Filter::reset(const float output)
{
    // For unity-DC-gain filters, a constant input/output steady state is:
    // u(k-1) = u(k-2) = output
    // y(k-1) = y(k-2) = output
    filter.u[0] = output;
    filter.u[1] = output;
    filter.y[0] = output;
    filter.y[1] = output;
}

// Assuming a constant input, differentiating results in zero output
// Currently only implemented for first order differentiators

void IIR_Filter::resetDifferentingFilterToZero(const float output)
{
    // Here "output" is interpreted as the current constant input level.
    filter.u[0] = output;
    filter.u[1] = output;
    filter.y[0] = 0.0f;
    filter.y[1] = 0.0f;
}

float IIR_Filter::apply(const float input)
{
    float output = 0.0f;

    if (filter.order == 1) {
        output = filter.B[0] * input + filter.B[1] * filter.u[0] - filter.A[0] * filter.y[0];
    } else if (filter.order == 2) {
        output = filter.B[0] * input + filter.B[1] * filter.u[0] + filter.B[2] * filter.u[1] -
                 filter.A[0] * filter.y[0] - filter.A[1] * filter.y[1];
    }

    // shift histories
    filter.u[1] = filter.u[0];
    filter.u[0] = input;

    filter.y[1] = filter.y[0];
    filter.y[0] = output;

    return output;
}

float IIR_Filter::applyConstrained(const float input, const float yMin, const float yMax)
{
    float outputUnconstrained = 0.0f;

    if (filter.order == 1) {
        outputUnconstrained = filter.B[0] * input + filter.B[1] * filter.u[0] - filter.A[0] * filter.y[0];
    } else if (filter.order == 2) {
        outputUnconstrained = filter.B[0] * input + filter.B[1] * filter.u[0] + filter.B[2] * filter.u[1] -
                              filter.A[0] * filter.y[0] - filter.A[1] * filter.y[1];
    }

    const float output = (outputUnconstrained < yMin)   ? yMin
                         : (outputUnconstrained > yMax) ? yMax
                                                        : outputUnconstrained;

    // shift histories, using the constrained output
    filter.u[1] = filter.u[0];
    filter.u[0] = input;

    filter.y[1] = filter.y[0];
    filter.y[0] = output;

    return output;
}

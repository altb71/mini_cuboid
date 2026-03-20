#pragma once

class IIR_Filter
{
public:
    IIR_Filter() {}; // default constructor
    virtual ~IIR_Filter();    // deconstructor

    float operator()(float u) { return evaluate((float)u); }

    void lowPass1Init(float, float);
    void differentiatingLowPass1Init(float, float);
    float evaluate(float);
    void reset(float, float);

private:
    float m_b0{0.0f};      // b0
    float m_b1{0.0f};      // b1
    float m_a1{0.0f};      // a1, a0 is always 1.0
    float m_u_kmin1{0.0f}; // u(k-1)
    float m_y_kmin1{0.0f}; // y(k-1)
};

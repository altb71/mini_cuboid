#pragma once

class LinearCharacteristics
{
public:
    LinearCharacteristics() {};                        // default constructor
    LinearCharacteristics(float, float);               // constructor with gain and offset
    LinearCharacteristics(float, float, float, float); // constructor with min/max values
    virtual ~LinearCharacteristics();                  // deconstructor

    void init(float, float);               // set gain and offset
    void init(float, float, float, float); // set gain and offset based on min/max values
    float evaluate(float);                 // calculate y(x)
    void set_limits(float, float);

private:
    float m_gain;
    float m_offset;
    float m_ulim;
    float m_llim;
};

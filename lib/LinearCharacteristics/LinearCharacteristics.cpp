#include "LinearCharacteristics.h"

using namespace std;

LinearCharacteristics::LinearCharacteristics(float gain, float offset)
{
    init(gain, offset);
}

LinearCharacteristics::LinearCharacteristics(float xmin, float xmax, float ymin, float ymax)
{
    init(xmin, xmax, ymin, ymax);
}

LinearCharacteristics::~LinearCharacteristics() {}

void LinearCharacteristics::init(float gain, float offset)
{
    m_gain = gain;
    m_offset = offset;
}

void LinearCharacteristics::init(float xmin, float xmax, float ymin, float ymax)
{
    m_gain = (ymax - ymin) / (xmax - xmin);
    m_offset = xmin - ymin / m_gain;
    m_ulim = 999999.0f;  // a large number
    m_llim = -999999.0f; // a large neg. number
    // TODO: Check if we do not want to set the limits here
    // m_ulim = ymax;
    // m_llim = ymin;
}

float LinearCharacteristics::evaluate(float x)
{
    float ret_val = m_gain * (x - m_offset);
    if (ret_val > m_ulim)
        ret_val = m_ulim;
    else if (ret_val < m_llim)
        ret_val = m_llim;

    return ret_val;
}

void LinearCharacteristics::set_limits(float ll, float ul)
{
    m_llim = ll;
    m_ulim = ul;
}

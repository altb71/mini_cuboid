#include "LinearCharacteristics.h"

using namespace std;

LinearCharacteristics::LinearCharacteristics(float g,float o){    // standard lin characteristics
    this->gain = g;
    this->offset = o;
    this->ulim = 999999.0;          // a large number
    this->llim = -999999.0;         // a large neg. number
}
LinearCharacteristics::LinearCharacteristics(float xmin,float xmax,float ymin, float ymax){    // standard lin characteristics
    this->gain = (ymax - ymin)/(xmax - xmin);
    this->offset = xmax - ymax/gain;
    this->ulim = 999999.0;          // a large number
    this->llim = -999999.0;         // a large neg. number
}


LinearCharacteristics::~LinearCharacteristics() {}


float LinearCharacteristics::evaluate(float x)
{   
    // calculate result as y(x) = gain * (x-offset)
return gain*(x - offset);
}

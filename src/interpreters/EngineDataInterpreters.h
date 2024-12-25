#ifndef _ENGINEDATAINTERPRETERS_H
#define _ENGINEDATAINTERPRETERS_H

#include "sensesp/transforms/curveinterpolator.h"

namespace sensesp {

////////////////////////////////////////////////////////////////////////
/*
  Curve interpolator transforms for 4-20mA inputs on the ADS1115 ADC's
  ====================================================================
      V = ADC_Count * (4.096v /32768) = [ADC_Count * 0.000125]

      4-20mA @ 165 Ohms
      0.66v to 3.3v
      ADC_Count RANGE: 5280 to 26400
*/

class DieselTankLevelInterpreter : public CurveInterpolator {
 public:
  DieselTankLevelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the *linear* ADC readings 
    // returned by the fuel tank level sender to a known percent value (ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownLevel));
    add_sample(CurveInterpolator::Sample(0, 0.00));
    add_sample(CurveInterpolator::Sample(5280, 0.00));
    // Full range of 4-20mA input
    add_sample(CurveInterpolator::Sample(26400, 100.0));
    add_sample(CurveInterpolator::Sample(32767, 100.0));
  }
};

class OilPressureInterpreter : public CurveInterpolator {
 public:
  OilPressureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the *linear* ADC readings 
    // returned by the oil pressure sender (0-100 PSI) to a known Pascal value (Pa)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownPa));
    add_sample(CurveInterpolator::Sample(0, 0.00));
    add_sample(CurveInterpolator::Sample(5280, 0.00));
    // Full range of 4-20mA input
    add_sample(CurveInterpolator::Sample(26400, 689475.72931783)); // Too much precision? LOL
    add_sample(CurveInterpolator::Sample(32767, 689475.72931783));
  }
};

}

#endif // _ENGINEDATAINTERPRETERS_H
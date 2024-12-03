#ifndef _RTDTEMPERATURETRANSMITTERINTERPRETERS_H
#define _RTDTEMPERATURETRANSMITTERINTERPRETERS_H

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

class RTD_ZeroTo100C_TempInterpreter : public CurveInterpolator {
 public:
  RTD_ZeroTo100C_TempInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the *linear* ADC readings 
    // returned by the RTD temperature sender to a known kelvin value (K)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownKelvinTemp));
    add_sample(CurveInterpolator::Sample(0, 273.15));
    add_sample(CurveInterpolator::Sample(5280, 273.15));
    add_sample(CurveInterpolator::Sample(26400, 373.15));
    add_sample(CurveInterpolator::Sample(32767, 373.15));
  }
};

class RTD_Minus50To150C_TempInterpreter : public CurveInterpolator {
 public:
  RTD_Minus50To150C_TempInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the *linear* ADC readings 
    // returned by the RTD temperature sender to a known kelvin value (K)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownKelvinTemp));
    add_sample(CurveInterpolator::Sample(0, 223.15));
    add_sample(CurveInterpolator::Sample(5280, 223.15));
    add_sample(CurveInterpolator::Sample(26400, 423.15));
    add_sample(CurveInterpolator::Sample(32767, 423.15));
  }
};

}

#endif // _RTDTEMPERATURETRANSMITTERINTERPRETERS_H
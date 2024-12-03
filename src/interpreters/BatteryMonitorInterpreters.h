#ifndef _BATTERYMONITORINTERPRETERS_H
#define _BATTERYMONITORINTERPRETERS_H

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

class BatteryVoltageInterpreter : public CurveInterpolator {
 public:
  BatteryVoltageInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the *linear* ADC readings 
    // returned by the battery voltage sender to a voltage value in volts (V)

    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownVoltage));
    add_sample(CurveInterpolator::Sample(0, 0.00));
    add_sample(CurveInterpolator::Sample(5280, 0.00));
    add_sample(CurveInterpolator::Sample(26400, 20.0));
    add_sample(CurveInterpolator::Sample(32767, 20.0));
  }
};

class BatteryCurrentInterpreter : public CurveInterpolator {
 public:
  BatteryCurrentInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the *linear* ADC readings 
    // returned by the battery current sender to a current value in amps (A)

    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownCurrent));
    add_sample(CurveInterpolator::Sample(0, -300.00));
    add_sample(CurveInterpolator::Sample(5280, -300.00));
    add_sample(CurveInterpolator::Sample(26400, 300.0));
    add_sample(CurveInterpolator::Sample(32767, 300.0));
  }
};

}

#endif // _BATTERYMONITORINTERPRETERS_H
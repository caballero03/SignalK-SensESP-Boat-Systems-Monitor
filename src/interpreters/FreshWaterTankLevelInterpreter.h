#ifndef _FRESHWATERTANKLEVELINTERPRETER_H
#define _FRESHWATERTANKLEVELINTERPRETER_H

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

class WaterTankLevelInterpreter : public CurveInterpolator {
 public:
  WaterTankLevelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the non-linear readings 
    // returned by the water tank level sender to a percent of fullness value (ratio)
    // Water level sensor has 1 meter range. Tank depth is 0.5 meters when full.
    
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownADCValue, knownPercent));
    add_sample(CurveInterpolator::Sample(0, 0.00));
    add_sample(CurveInterpolator::Sample(5280, 0.00));
    add_sample(CurveInterpolator::Sample(5984, 6.25));
    add_sample(CurveInterpolator::Sample(6688, 12.50));
    add_sample(CurveInterpolator::Sample(7392, 18.75));
    add_sample(CurveInterpolator::Sample(8096, 25.00));
    add_sample(CurveInterpolator::Sample(8800, 31.25));         // TODO: Change this to a 0.0 to 1.0 ratio
    add_sample(CurveInterpolator::Sample(9504, 37.50));
    add_sample(CurveInterpolator::Sample(10208, 43.75));
    add_sample(CurveInterpolator::Sample(10912, 50.00));
    add_sample(CurveInterpolator::Sample(11616, 56.25));
    add_sample(CurveInterpolator::Sample(12320, 62.50));
    add_sample(CurveInterpolator::Sample(13024, 68.75));
    add_sample(CurveInterpolator::Sample(13728, 75.00));
    add_sample(CurveInterpolator::Sample(14432, 81.25));
    add_sample(CurveInterpolator::Sample(15136, 87.50));
    add_sample(CurveInterpolator::Sample(15840, 93.75)); // half of sensor range
    add_sample(CurveInterpolator::Sample(32767, 100.0));
  }
};

}

#endif // _FRESHWATERTANKLEVELINTERPRETER_H
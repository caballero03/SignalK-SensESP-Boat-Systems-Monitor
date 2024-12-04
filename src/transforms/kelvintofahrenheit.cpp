#include "kelvintofahrenheit.h"

sensesp::KelvinToFahrenheit::KelvinToFahrenheit(String config_path) :
   FloatTransform(config_path) {
}


void sensesp::KelvinToFahrenheit::set(const float& K) {
    float output = 9.0 / 5.0 * (K - 273.15) + 32.0;
    this->emit(output);
}

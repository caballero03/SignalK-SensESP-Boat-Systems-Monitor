#ifndef _kelvinToFahrenheit_H
#define _kelvinToFahrenheit_H

#include "sensesp/transforms/transform.h"

namespace sensesp {

class KelvinToFahrenheit : public FloatTransform {

    public:
        KelvinToFahrenheit(String config_path="");

        virtual void set(const float& K) override;
};

}

#endif // _kelvinToFahrenheit_H

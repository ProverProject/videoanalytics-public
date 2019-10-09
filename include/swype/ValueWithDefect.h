//
// Created by babay on 06.01.2018.
//

#ifndef PROVER_VALUEWITHDEFECT_H
#define PROVER_VALUEWITHDEFECT_H


#include <math.h>

class ValueWithDefect {
public:

    ValueWithDefect() : _value(0), _defect(0) {}

    ValueWithDefect(double value, float defect) : _value(value), _defect(defect) {}

    inline double Value() const {
        return _value;
    }

    inline float Defect() const {
        return _defect;
    }

    inline void Set(double value, float defect){
        _value = value;
        _defect = defect;
    }

    inline ValueWithDefect operator/(ValueWithDefect divider) {
        if (divider._value == 0){
            return ValueWithDefect(HUGE_VAL, HUGE_VALF);
        }
        ValueWithDefect res;
        res._value = _value / divider._value;
        float t1 = (float) (_defect / divider._value);
        float t2 = (float) (_value * divider._defect / divider._value / divider._value);
        res._defect = sqrtf(t1 * t1 + t2 * t2);
        return res;
    }

    inline ValueWithDefect operator*(ValueWithDefect y) {
        ValueWithDefect res;
        res._value = _value * y._value;
        float t1 = _defect * (float) y._value;
        float t2 = y._defect * (float) _value;
        res._defect = sqrtf(t1 * t1 + t2 * t2);
        return res;
    }

    inline void operator*=(ValueWithDefect y) {
        float t1 = _defect * (float) y._value;
        float t2 = y._defect * (float) _value;
        _defect = sqrtf(t1 * t1 + t2 * t2);
        _value *= y._value;
    }

    inline void operator/=(double div) {
        _value /= div;
        _defect /= div;
    }

private:
    double _value;
    float _defect;
};


#endif //PROVER_VALUEWITHDEFECT_H

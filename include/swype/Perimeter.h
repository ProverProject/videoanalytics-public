//
// Created by babay on 06.01.2018.
//

#ifndef PROVER_PERIMETER_H
#define PROVER_PERIMETER_H


#include "swype/VectorExplained.h"

class Perimeter {
public:
    Perimeter() : _perimeter(0), _defect2(0) {}

    inline void Add(VectorExplained v) {
        _perimeter += v.Mod();
        float t = v.ModDefect();
        _defect2 += t * t;
    }

    double Value() const {
        return _perimeter;
    }

    inline double Defect() const {
        return sqrt(_defect2);
    }

private:
    double _perimeter = 0;
    double _defect2 = 0;
};


#endif //PROVER_PERIMETER_H

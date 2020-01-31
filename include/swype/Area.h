//
// Created by babay on 06.01.2018.
//

#ifndef PROVER_AREA_H
#define PROVER_AREA_H


#include "swype/VectorExplained.h"

class Area {
public:
    Area(const VectorExplained &v) : _area(0),
                                     _defect2(0),
                                     sum(v),
                                     sumPrev(v) {}

    inline void AppendVector(const VectorExplained &v) {
        sum.Add(v);
        float triangleArea = (float) (sum.X() * sumPrev.Y() - sum.Y() * sumPrev.X()) / 2;
        _area += triangleArea;

        double t1 = (sum.X() * sumPrev.DefectY());
        double t2 = (sum.DefectX() * sumPrev.Y());
        double t3 = (sum.Y() * sumPrev.DefectX());
        double t4 = (sum.DefectY() * sumPrev.X());

        _defect2 += t1 * t1 + t2 * t2 + t3 * t3 + t4 * t4;
        sumPrev = sum;
    }

    inline double GetDefect() const {
        return sqrt(_defect2);
    }

    double Get() const {
        return fabs(_area);
    }

    const VectorExplained &Sum() const {
        return sum;
    }

private:
    double _area;
    double _defect2;
    VectorExplained sum;
    VectorExplained sumPrev;
};


#endif //PROVER_AREA_H

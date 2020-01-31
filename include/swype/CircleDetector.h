//
// Created by babay on 08.12.2017.
//

#ifndef PROVER_SWYPECIRCLEDETECTOR_H
#define PROVER_SWYPECIRCLEDETECTOR_H


#include "swype/VectorExplained.h"
#include "swype/ValueWithDefect.h"

#define SHIFTS 128

#define MAX_CIRCLE_DURATION_MS 2000

#define MIN_CIRCLE_AREA 0.14
#define MIN_REPORTABLE_CLOSED_POLYLINE 0.02
#define MAX_DEVIATION 0.09
#define MIN_AREA_BY_P2_TO_CIRCLE 0.67

class CircleDetector {
public:
    enum Result {
        /**
         * nothing like circle detected
         */
                NoCircle = 0,
        /**
         * good circle detected
         */
                GotCircle = 1,
        /**
         * Area of detected closed curve was too small
         */
                AreaTooSmall = 2,
        /**
         * detected closed curve is not round enought
         */
                CurveNotRoundEnough = 4,
    };

    CircleDetector() :
            _pos(0),
            _total(0),
            _minCircleArea(MIN_CIRCLE_AREA),
            _maxDeviation(MAX_DEVIATION),
            _minAreaByP2toCircle(MIN_AREA_BY_P2_TO_CIRCLE),
            _relaxed(false) {}

    void AddShift(const VectorExplained &shift);

    bool IsCircle() const;

    /**
     * check if there is a circle or closed curve;
     * @param curveCoordinates - [out] closed curve coordinates; coordinates are normalized and shifted; (0,0) is a curve center
     * @param coordinatesArrayLength - [in] coordinates array length
     * @param writtenCoordinates - [out] coordinates count  = (curve section count + 1) * 2, *cause start might be a little different from end
     * @return
     */
    Result
    CheckCircle(float *curveCoordinates, int coordinatesArrayLength, int &writtenCoordinates) const;

    /**
     * check if there is a circle or closed curve
     * @param curveLength - [out] closed curve section count
     * @return
     */
    Result CheckCircle(int &curveLength) const;

    void SetRelaxed(bool relaxed);

    void Clear() {
        _pos = 0;
        _total = 0;
    }

private:
    ValueWithDefect CalculateArea(int amount, ValueWithDefect &perimeter) const;

    VectorExplained _shifts[SHIFTS];
    int _pos;
    int _total;

    double _minCircleArea;
    double _maxDeviation;
    double _minAreaByP2toCircle;
    bool _relaxed;
};


#endif //PROVER_SWYPECIRCLEDETECTOR_H

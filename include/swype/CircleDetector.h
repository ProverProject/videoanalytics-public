//
// Created by babay on 08.12.2017.
//

#ifndef PROVER_MVP_ANDROID_SWIPECIRCLEDETECTOR_H
#define PROVER_MVP_ANDROID_SWIPECIRCLEDETECTOR_H


#include "swype/VectorExplained.h"
#include "swype/ValueWithDefect.h"

#define SHIFTS 64

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

    void Reset() {
        pos_ = 0;
        total_ = 0;
    }

    void SetRelaxed(bool relaxed);

    void Clear();

private:
    ValueWithDefect CalculateArea(int amount, ValueWithDefect &perResult) const;

    VectorExplained shifts_[SHIFTS];
    int pos_ = 0;
    int total_ = 0;

    double _minCircleArea = MIN_CIRCLE_AREA;
    double _maxDeviation = MAX_DEVIATION;
    double _minAreaByP2toCircle = MIN_AREA_BY_P2_TO_CIRCLE;
    bool _relaxed = true;
};


#endif //PROVER_MVP_ANDROID_SWIPECIRCLEDETECTOR_H

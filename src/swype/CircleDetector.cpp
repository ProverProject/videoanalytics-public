//
// Created by babay on 08.12.2017.
//

#include "swype/CircleDetector.h"
#include "swype/common.h"
#include "swype/Perimeter.h"
#include "swype/Area.h"
#include "swype/ValueWithDefect.h"

// 0.25 / PI
#define CIRCLE_S_BY_P2 0.07957747154594766788444188168626

void CircleDetector::AddShift(const VectorExplained &shift) {
    shifts_[pos_] = shift;
    pos_ = (pos_ + 1) % SHIFTS;
    ++total_;
    if (total_ > SHIFTS)
        total_ = SHIFTS;
}

bool CircleDetector::IsCircle() const {
    int temp;
    return CheckCircle(temp) == Result::GotCircle;
}


CircleDetector::Result
CircleDetector::CheckCircle(float *curveCoordinates, int coordinatesArrayLength,
                            int &writtenCoordinates) const {
    int sections = 0;
    auto result = CheckCircle(sections);
    if (sections > 0 && (sections * 2 + 2 <= coordinatesArrayLength)) {
        Vector current(0, 0);
        Vector sum(0, 0);
        int shiftPos;
        int coordinatePos = sections * 2;
        curveCoordinates[coordinatePos] = 0;
        curveCoordinates[coordinatePos + 1] = 0;
        for (int i = 1; i <= sections; ++i) {
            shiftPos = (pos_ - i + SHIFTS) % SHIFTS;
            current += shifts_[shiftPos];
            sum += current;
            curveCoordinates[--coordinatePos] = static_cast<float>(current.Y());
            curveCoordinates[--coordinatePos] = static_cast<float>(current.X());
        }
        sum *= (1.0 / (sections + 1));
        coordinatePos = -1;
        for (int i = 0; i <= sections; ++i) {
            curveCoordinates[++coordinatePos] -= sum.X();
            curveCoordinates[++coordinatePos] -= sum.Y();
        }
        writtenCoordinates = sections * 2 + 2;
    } else {
        writtenCoordinates = 0;
    }
    return result;
}


CircleDetector::Result CircleDetector::CheckCircle(int &curveLength) const {
    int result = Result::NoCircle;
    curveLength = 0;

    int pos = (pos_ - 1 + SHIFTS) % SHIFTS;
    VectorExplained sum = shifts_[pos];

    unsigned int timestamp = shifts_[pos].Timestamp();
    unsigned int noFramesBefore = timestamp < MAX_CIRCLE_DURATION_MS ?
                                  0 : timestamp - MAX_CIRCLE_DURATION_MS;

    double minDeviation = 10;
    double minDeviationDefect = 0;
    double minDeviationDist = 0;

    for (int i = 2; i <= total_; i++) {
        pos = (pos_ - i + SHIFTS) % SHIFTS;
        if (shifts_[pos].Timestamp() < noFramesBefore) {
            if ((logLevel & LOG_CIRCLE_DETECTION) && minDeviation < 0.5) {
                LOGI_NATIVE("IsCircle minDeviation: %.4f-%.4f = %.4f", minDeviation,
                            minDeviationDefect, minDeviationDist);
            }
            return static_cast<Result>(result);
        }

        sum.Add(shifts_[pos]);
        if (i > 5) {
            double dist = _relaxed ? sum.Mod() - sum.ModDefect() : sum.Mod();
            if (dist < _maxDeviation) {
                //here the curve is definitely closed
                ValueWithDefect perimeter;
                ValueWithDefect area = CalculateArea(i, perimeter);
                ValueWithDefect areaByP2ToCircle = area / (perimeter * perimeter);
                areaByP2ToCircle /= CIRCLE_S_BY_P2;

                double areaVal = _relaxed ? area.Value() + area.Defect() : area.Value();
                double aToPValue = _relaxed ? areaByP2ToCircle.Value() + areaByP2ToCircle.Defect()
                                            : areaByP2ToCircle.Value();

                if (logLevel & LOG_CIRCLE_DETECTION) {
                    bool gotCircle = areaVal >= _minCircleArea && aToPValue >= _minAreaByP2toCircle;
                    LOGI_NATIVE(
                            "IsCircle #%d vertices: %d, diff: %.4f-+%.4f (%.4f, %.4f) , area: %.4f+%.4f, areaByP2 to target: %.4f+%.4f, got: %d",
                            timestamp, i + 1, sum.Mod(), sum.ModDefect(), sum.DefectX(), sum.DefectY(),
                            area.Value(), area.Defect(), areaByP2ToCircle.Value(),
                            areaByP2ToCircle.Defect(), gotCircle);
                }

                if (areaVal >= _minCircleArea && aToPValue >= _minAreaByP2toCircle) {
                    curveLength = i;
                    return Result::GotCircle;
                }
                if (areaVal > MIN_REPORTABLE_CLOSED_POLYLINE && curveLength == 0) {
                    curveLength = i;
                    if (areaVal < _minCircleArea)
                        result |= Result::AreaTooSmall;
                    if (aToPValue < _minAreaByP2toCircle)
                        result |= Result::CurveNotRoundEnough;
                }
            }
            if (sum.Mod() < minDeviation) {
                minDeviation = sum.Mod();
                minDeviationDefect = sum.ModDefect();
                minDeviationDist = dist;
            }
        }
    }
    return static_cast<Result>(result);
}

ValueWithDefect CircleDetector::CalculateArea(int amount, ValueWithDefect &perimeter) const {
    Perimeter perimeterCalc;
    Area area(shifts_[pos_]);

    for (int i = 2; i <= amount; i++) {
        int pos = (pos_ - i + SHIFTS) % SHIFTS;
        perimeterCalc.Add(shifts_[pos]);
        area.AppendVector(shifts_[pos]);
    }
    VectorExplained last(area.sum.X() - shifts_[pos_].X(), area.sum.Y() - shifts_[pos_].Y());
    perimeterCalc.Add(last);
    area.AppendVector(last);

    perimeter.Set(perimeterCalc._perimeter, (float) perimeterCalc.GetDefect());
    return ValueWithDefect(fabs(area._area), (float) area.GetDefect());
}

void CircleDetector::SetRelaxed(bool relaxed) {
    _relaxed = relaxed;
    if (relaxed) {
        _minCircleArea = MIN_CIRCLE_AREA / 1.2;
        _maxDeviation = MAX_DEVIATION;
        _minAreaByP2toCircle = MIN_AREA_BY_P2_TO_CIRCLE / 1.2;
    } else {
        _minCircleArea = MIN_CIRCLE_AREA;
        _maxDeviation = MAX_DEVIATION;
        _minAreaByP2toCircle = MIN_AREA_BY_P2_TO_CIRCLE;
    }
}

void CircleDetector::Clear() {
    pos_ = 0;
    total_ = 0;
}
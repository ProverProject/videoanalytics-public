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
    _shifts[_pos] = shift;
    _pos = (_pos + 1) % SHIFTS;
    ++_total;
    if (_total > SHIFTS)
        _total = SHIFTS;
}

bool CircleDetector::IsCircle(float &quality) const {
    int temp;
    return CheckCircle(temp, quality) == Result::GotCircle;
}


CircleDetector::Result
CircleDetector::CheckCircle(float *curveCoordinates, int coordinatesArrayLength,
                            int &writtenCoordinates) const {
    int sections = 0;
    float quality;
    auto result = CheckCircle(sections, quality);
    if (sections > 0 && (sections * 2 + 2 <= coordinatesArrayLength)) {
        Vector current(0, 0);
        Vector sum(0, 0);
        int shiftPos;
        int coordinatePos = sections * 2;
        curveCoordinates[coordinatePos] = 0;
        curveCoordinates[coordinatePos + 1] = 0;
        for (int i = 1; i <= sections; ++i) {
            shiftPos = (_pos - i + SHIFTS) % SHIFTS;
            current += _shifts[shiftPos];
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

CircleDetector::Result CircleDetector::CheckCircle(int &curveLength, float &quality) const {
    int result = Result::NoCircle;
    curveLength = 0;

    int pos = (_pos - 1 + SHIFTS) % SHIFTS;
    VectorExplained sum = _shifts[pos];

    unsigned int timestamp = _shifts[pos].Timestamp();
    unsigned int noFramesBefore = timestamp < MAX_CIRCLE_DURATION_MS ?
                                  0 : timestamp - MAX_CIRCLE_DURATION_MS;

    double minDeviation = 10;
    double minDeviationDefect = 0;
    double minDeviationDist = 0;

    for (int i = 2; i <= _total; i++) {
        pos = (_pos - i + SHIFTS) % SHIFTS;
        if (_shifts[pos].Timestamp() < noFramesBefore) {
            if ((logLevel & LOG_CIRCLE_DETECTION) && minDeviation < 0.5) {
                LOGI_NATIVE("IsCircle minDeviation: %.4f-%.4f = %.4f", minDeviation,
                            minDeviationDefect, minDeviationDist);
            }
            return static_cast<Result>(result);
        }

        sum.Add(_shifts[pos]);
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
                    float q = gotCircle ? std::max(Quality(area, MIN_CIRCLE_AREA), Quality(areaByP2ToCircle, MIN_AREA_BY_P2_TO_CIRCLE)) : 0;
                    LOGI_NATIVE(
                            "IsCircle #%d vertices: %d, diff: %.4f-+%.4f (%.4f, %.4f) , area: %.4f+%.4f, areaByP2 to target: %.4f+%.4f, got: %d, q: %.3f",
                            timestamp, i + 1, sum.Mod(), sum.ModDefect(), sum.DefectX(),
                            sum.DefectY(),
                            area.Value(), area.Defect(), areaByP2ToCircle.Value(),
                            areaByP2ToCircle.Defect(), gotCircle, q);
                }

                if (areaVal >= _minCircleArea && aToPValue >= _minAreaByP2toCircle) {
                    if (_relaxed) {
                        quality = std::max(Quality(area, MIN_CIRCLE_AREA), Quality(areaByP2ToCircle, MIN_AREA_BY_P2_TO_CIRCLE));
                    }
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
    Area area(_shifts[_pos]);

    for (int i = 2; i <= amount; i++) {
        int pos = (_pos - i + SHIFTS) % SHIFTS;
        perimeterCalc.Add(_shifts[pos]);
        area.AppendVector(_shifts[pos]);
    }
    VectorExplained last(area.Sum().X() - _shifts[_pos].X(), area.Sum().Y() - _shifts[_pos].Y());
    perimeterCalc.Add(last);
    area.AppendVector(last);

    perimeter.Set(perimeterCalc.Value(), (float) perimeterCalc.Defect());
    return ValueWithDefect(area.Get(), (float) area.GetDefect());
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


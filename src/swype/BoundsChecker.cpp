//
// Created by babay on 09.12.2017.
//

#include "swype/BoundsChecker.h"
#include "swype/common.h"
#include "swype/VectorExplained.h"

#define SQRT2 1.4142135623730950488016887242097

void BoundsChecker::SetDirection(int targetDirection) {
    _isDiagonal = targetDirection % 2 == 0;
    SetMatForDirection(targetDirection);
    _target.SetDirection(_isDiagonal ? 8 : 7);
}

void BoundsChecker::SetTurnMatForDirectionDiff(int directionDiff) {
    switch (directionDiff) {
        case 0:
            _turnMat[0][0] = _turnMat[1][1] = 1;
            _turnMat[1][0] = _turnMat[0][1] = 0;
            break;
        case -4:
            _turnMat[0][0] = _turnMat[1][1] = -1;
            _turnMat[1][0] = _turnMat[0][1] = 0;
            break;
        case 2:
            _turnMat[0][0] = _turnMat[1][1] = 0;
            _turnMat[1][0] = 1;
            _turnMat[0][1] = -1;
            break;
        case -2:
            _turnMat[0][0] = _turnMat[1][1] = 0;
            _turnMat[1][0] = -1;
            _turnMat[0][1] = 1;
            break;
    }
}


void BoundsChecker::SetMatForDirection(int direction) {
    _turnMat[1][0] = _turnMat[0][1] = 0;
    switch (direction) {
        case 8:
        case 7:
            _turnMat[0][0] = _turnMat[1][1] = 1;
            _turnMat[1][0] = _turnMat[0][1] = 0;
            break;
        case 2:
        case 3:
            _turnMat[0][0] = -1;
            _turnMat[1][1] = 1;
            _turnMat[1][0] = _turnMat[0][1] = 0;
            break;
        case 4:
            _turnMat[0][0] = -1;
            _turnMat[1][1] = -1;
            _turnMat[1][0] = _turnMat[0][1] = 0;
            break;
        case 6:
            _turnMat[0][0] = 1;
            _turnMat[1][1] = -1;
            _turnMat[1][0] = _turnMat[0][1] = 0;
            break;
        case 1:
            _turnMat[0][0] = _turnMat[1][1] = 0;
            _turnMat[1][0] = 1;
            _turnMat[0][1] = 1;
            break;
        case 5:
            _turnMat[0][0] = _turnMat[1][1] = 0;
            _turnMat[1][0] = 1;
            _turnMat[0][1] = -1;
            break;
    }
}

/**
 *
 * @param current -- current position vector
 * @return true if we're still OK
 */
bool BoundsChecker::CheckBounds(VectorExplained p) {
    // turn _current so we should move to +x (of +x, +y for Diagonal)
    // fail if we've got too close to another swype-point
    p.MulWithDefect(_turnMat);

    if (p.DistanceTo(_target) < BOUNDS_CHECKER_TARGET_SAFE_ZONE) {
        if (logLevel & LOG_GENERAL_DETECTION) {
            LOGI_NATIVE("Bounds within target safe zone: %.4f, %.4f, distToTarget: %.4f",
                        p.X(), p.Y(), p.DistanceTo(_target));
        }
        return true;
    }

    float tr1 = 1 + _targetRadius;

    if (_isDiagonal) {
        if (p.X() < -FIT_FACTOR_H || p.Y() < -FIT_FACTOR_H || p.X() > tr1 || p.Y() > tr1) {
            if (logLevel & LOG_GENERAL_DETECTION) {
                LOGI_NATIVE("Bounds_f1 %.4f, %.4f, tr: %.4f", p.X(), p.Y(), _targetRadius);
            }
            return false;
        }

        if (p.X() + p.Y() <= 0) {
            return true;
        }

        if (p.Y() > p.X()) {
            p.FlipXY();
        }

        // ensure that we can't get into wrong swype-point (1,0) accounting defect
        // defect is double 'cause server can will have different result
#ifdef RECT_DEFECT
        Vector shifted = p.ShiftDefectRectToPointMagnet(1, 0, 2);
#else
        Vector shifted = p.ShiftDefectEllipseToPointMagnet(1, 0, 2);
#endif
        float distanceToWrongPoint = (float) shifted.DistanceTo(1, 0);
        if (distanceToWrongPoint <= _targetRadiusOther) {
            if (logLevel & LOG_GENERAL_DETECTION) {
                LOGI_NATIVE("Bounds 2shifted (%.4f %.4f) dist = %.4f",
                            shifted.X(), shifted.Y(), distanceToWrongPoint);
            }
            return false;
        }

        //distance to line x = y -- it is the line that goes to target point
        double d1 = (p.X() - p.Y()) / SQRT2;
        // distance to remaining non-target swype point
        double r2 = p.DistanceTo(1, 0);
        if (!(logLevel & LOG_GENERAL_DETECTION))
            return d1 < r2;
        else {
            if (d1 < r2)
                return true;
            LOGI_NATIVE("Bounds_f3 %.4f, %.4f, %f, %f", p.X(), p.Y(), d1, r2);
            return false;
        }
    } else {
        return p.X() >= -FIT_FACTOR_H && p.X() <= tr1 && fabs(p.Y()) <= FIT_FACTOR_H;
    }
}


bool BoundsChecker::CheckBoundsWithDefect(VectorExplained p) {
    p.MulWithDefect(_turnMat);

    if (p.MinDistanceToWithDefect(_target) < BOUNDS_CHECKER_TARGET_SAFE_ZONE) {
        if (logLevel & LOG_GENERAL_DETECTION) {
            LOGI_NATIVE("Bounds within target safe zone: %.4f, %.4f, distToTarget: %.4f",
                        p.X(), p.Y(), p.MinDistanceToWithDefect(_target));
        }
        return true;
    }

    float tr1 = 1 + _targetRadius;

    if (_isDiagonal) {
        if (!p.CheckWithinRectWithDefect(-FIT_FACTOR_H_RELAXED, -FIT_FACTOR_H_RELAXED, tr1, tr1)) {
            if (logLevel & LOG_GENERAL_DETECTION) {
                LOGI_NATIVE("Bounds_f1 %.4f, %.4f", p.X(), p.Y());
            }
            return false;
        }

        if (p.X() + p.Y() <= 0) {
            return true;
        }

        if (p.X() < p.Y()) {
            p.FlipXY();
        }

        // ensure that we can't get into wrong swype-point (1,0) accounting defect
#ifdef RECT_DEFECT
        Vector shifted = p.ShiftDefectRectToPointMagnet(1, 0, 1);
#else
        Vector shifted = p.ShiftDefectEllipseToPointMagnet(1, 0, 1);
#endif
        float distanceToWrongPoint = (float) shifted.DistanceTo(1, 0);
        if (distanceToWrongPoint <= _targetRadiusOther) {
            if (logLevel & LOG_GENERAL_DETECTION) {
                LOGI_NATIVE("Bounds 2shifted (%.4f %.4f) dist = %.4f", shifted.X(), shifted.Y(),
                            distanceToWrongPoint);
            }
            return false;
        }
#ifdef RECT_DEFECT
        Vector shiftedToLine = p.ShiftDefectRectToTouchLineMagnet();
#else
        Vector shiftedToLine = p.ShiftDefectEllipseToTouchLineMagnet();
#endif
        if (shiftedToLine.X() == shiftedToLine.Y())
            return true;
        //a point on the diagonal within defect area, so we are definitely not failed
        if (p.X() <= p.Y()) {
            return true;
        }

        //distance to line x = y -- it is the line that goes to target point
        double d1 = (shiftedToLine.X() - shiftedToLine.Y()) / SQRT2;
        // distance to remaining non-target swype point
        double r2 = shiftedToLine.DistanceTo(1, 0);
        if (logLevel & LOG_GENERAL_DETECTION) {
            if (d1 < r2)
                return true;
            LOGI_NATIVE("Bounds_f3 %.4f, %.4f, %f, %f", shiftedToLine.X(), shiftedToLine.Y(), d1,
                        r2);
            return false;
        } else return d1 < r2;
    } else {
        return p.CheckWithinRectWithDefect(-FIT_FACTOR_H_RELAXED, -FIT_FACTOR_H_RELAXED, tr1,
                                           FIT_FACTOR_H_RELAXED);
    }
}

void BoundsChecker::SetTargetRadius(float targetRadius, float targetRadiusOther) {
    _targetRadius = targetRadius;
    _targetRadiusOther = targetRadiusOther;
}
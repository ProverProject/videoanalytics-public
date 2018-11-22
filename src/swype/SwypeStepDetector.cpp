//
// Created by babay on 07.12.2017.
//

#include "swype/SwypeStepDetector.h"
#include "swype/common.h"
#include "swype/settings.h"

void SwypeStepDetector::Add(VectorExplained shift) {
    shift.Mul(_parameters._speedMultX, _parameters._speedMultY);

    _current.Add(shift);
    _current._timestamp = shift._timestamp;
    _total.Add(shift);
    _count++;
}

void SwypeStepDetector::Set(VectorExplained total) {
    total._x *= _parameters._speedMultX;
    total._defectX *= _parameters._speedMultX;
    total._y *= _parameters._speedMultY;
    total._defectY *= _parameters._speedMultY;

    _current = total;
    _count++;
}

void SwypeStepDetector::Reset() {
    _current.Reset();
    _target.Reset();
    _total.Reset();
    _count = 0;
}

void SwypeStepDetector::Configure(DetectorParameters parameters) {
    _parameters = parameters;
    _targetRadius = parameters._targetRadius;
}

void SwypeStepDetector::FinishStep() {
    _count = 0;
    _current -= _target;

    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("FinishStep %d (%f %f) ", _id, _current._x, _current._y);
    }
}

int SwypeStepDetector::CheckState(bool withDefect) {
    double distance = withDefect ? _current.MinDistanceToWithDefect(_target) :
                      _current.DistanceTo(_target);

    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE(
                "CheckState %d |(%+.4f %+.4f) - (%+.4f %+.4f)|= %.4f, total: |%+.4f+-%.4f %+.4f+-%.4f| = %.4f+-%.4f defSum |%.4f,%.4f|= %.4f",
                _id,
                _current._x, _current._y, _target._x, _target._y, distance,
                _total._x, _total._defectX, _total._y, _total._defectY, _total._mod,
                _total.ModDefect(), _current._defectX, _current._defectY, _current.ModDefect());
    }

    if (distance <= _targetRadius) {
        if (logLevel & LOG_GENERAL_DETECTION) {
            LOGI_NATIVE("CheckState %d reached ", _id);
        }
        return 1;
    }

    bool boundsCheckResult = withDefect ? _BoundsChecker.CheckBoundsWithDefect(_current)
                                        : _BoundsChecker.CheckBounds(_current);

    if (!boundsCheckResult) {
        if (logLevel & LOG_GENERAL_DETECTION) {
            LOGI_NATIVE("CheckState %d boundsCheck failing ", _id);
        }
    }

    return boundsCheckResult ? 0 : -1;
}

void SwypeStepDetector::SetDirection(int dir) {
    Reset();
    _target.SetDirection(dir);
    SetTarget(_target);
}


void SwypeStepDetector::AdvanceDirection(int dir) {
    FinishStep();
    _target.SetDirection(dir);
    SetTarget(_target);
}

void SwypeStepDetector::SetTarget(VectorExplained target) {
    _target = target;

    _targetRadius = _parameters._targetRadius;
    if (_parameters._relaxed && _target._direction % 2 == 0) {// for diagonal target at server
        _targetRadius *= DIAGONAL_TARGET_RADIUS_MULT;
    }

    _BoundsChecker.SetDirection(_target._direction);
    _BoundsChecker.SetTargetRadius(_targetRadius, _parameters._targetRadius);

    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("SetTarget %d (%.1f %.1f) d %d", _id, target._x, target._y, target._direction);
    }
}

bool
SwypeStepDetector::CheckCompleteStep(int direction, const std::vector<VectorExplained> &shifts) {
    SetDirection(direction);
    int state = 0;
    for (unsigned int i = 0; i < shifts.size(); ++i) {
        Add(shifts[i]);
        state = CheckState(_parameters._relaxed);
        if (state == -1)
            return false;
    }

    if (logLevel & LOG_GENERAL_DETECTION) {
        if (state == 1)
            LOGI_NATIVE("SwypeStepDetector::CheckCompleteStep: target reached");
        else
            LOGI_NATIVE("SwypeStepDetector::CheckCompleteStep: target not reached");
    }
    return state == 1;
}

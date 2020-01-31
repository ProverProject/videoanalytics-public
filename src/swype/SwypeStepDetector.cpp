//
// Created by babay on 07.12.2017.
//

#include "swype/SwypeStepDetector.h"
#include "swype/common.h"
#include "swype/settings.h"

void SwypeStepDetector::Add(VectorExplained shift) {
    shift.Mul(_parameters.SpeedMultX(), _parameters.SpeedMultY());

    _current.Add(shift);
    _current.SetTimestamp(shift.Timestamp());
    _total.Add(shift);
    _count++;
}

void SwypeStepDetector::Set(VectorExplained total) {
    total.Mul(_parameters.SpeedMultX(), _parameters.SpeedMultY());
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
    _targetRadius = parameters.TargetRadius();
}

void SwypeStepDetector::FinishStep() {
    _count = 0;
    _current -= _target;

    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("FinishStep %d (%f %f) ", _id, _current.X(), _current.Y());
    }
}

int SwypeStepDetector::CheckState(bool withDefect) {
    double distance = withDefect ? _current.MinDistanceToWithDefect(_target) :
                      _current.DistanceTo(_target);

    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE(
                "CheckState %d |(%+.4f %+.4f) - (%+.4f %+.4f)|= %.4f, total: |%+.4f+-%.4f %+.4f+-%.4f| = %.4f+-%.4f defSum |%.4f,%.4f|= %.4f",
                _id,
                _current.X(), _current.Y(), _target.X(), _target.Y(), distance,
                _total.X(), _total.DefectX(), _total.Y(), _total.DefectY(),
                _total.Mod(), _total.ModDefect(),
                _current.DefectX(), _current.DefectY(), _current.ModDefect());
    }

    if (distance <= _targetRadius) {
        if (logLevel & LOG_GENERAL_DETECTION) {
            LOGI_NATIVE("CheckState %d reached ", _id);
        }
        return 1;
    }

    bool boundsCheckResult = withDefect ? _boundsChecker.CheckBoundsWithDefect(_current)
                                        : _boundsChecker.CheckBounds(_current);

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

void SwypeStepDetector::SetTarget(const VectorExplained &target) {
    _target = target;

    _targetRadius = _parameters.TargetRadius();
    if (_parameters.IsRelaxed() && _target.Direction() % 2 == 0) {// for diagonal target at server
        _targetRadius *= DIAGONAL_TARGET_RADIUS_MULT;
    }

    _boundsChecker.SetDirection(_target.Direction());
    _boundsChecker.SetTargetRadius(_targetRadius, _parameters.TargetRadius());

    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("SetTarget %d (%.1f %.1f) d %d", _id, target.X(), target.Y(),
                    target.Direction());
    }
}

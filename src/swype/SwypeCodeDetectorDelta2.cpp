//
// Created by babay on 29.07.2019.
//

#include "swype/SwypeCodeDetectorDelta2.h"
#include "swype/DetectorState.h"

void SwypeCodeDetectorDelta2::SetNextStep(SwypeStep step) {
    _nextStep = step;
    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("set swype next step. num: %d, dir: %d, dur: %d",
                    step.number, step.direction, step.maxDurationMs);
    }
}

void SwypeCodeDetectorDelta2::NextFrame(const VectorExplained &shift) {
    _currentTimestamp = shift.Timestamp();
    if (_currentTimestamp < _startTimestamp) {
    } else if (_currentTimestamp > _maxTimestamp) {
        _status = -2;
    } else if (shift.Mod() <= 0) { // generally == 0
        _status = 0;
    } else {
        _stepDetector.Add(shift);
        _status = _stepDetector.CheckState(false);
        if (_status == 1 && _useSwypeCode && _currentStep.number + 1 >= _code.Length()) {
            _status = 2;
        }
    }
}

unsigned int SwypeCodeDetectorDelta2::TimeToFail() const {
    return _maxTimestamp - _currentTimestamp;
}

void SwypeCodeDetectorDelta2::Reset(bool resetSwypeCode) {
    SwypeCodeDetector::Reset(resetSwypeCode);
    _nextStep = {0, 0, 0};
    if (!resetSwypeCode && _useSwypeCode) {
        SwypeCode code = _code;
        SetSwypeCode(code);
    } else {
        _useSwypeCode = false;
    }
}

void SwypeCodeDetectorDelta2::AdvanceStep() {
    if (_useSwypeCode) {
        unsigned int duration = _maxTimestamp - _currentTimestamp;
        unsigned int num = _currentStep.number + 1;
        SetCurrentStep(SwypeStep{num, _code.DirectionAt(num), duration}, _currentTimestamp);
    } else {
        SetCurrentStep(_nextStep, _currentTimestamp);
        _nextStep = {0, 0, 0};
    }
}

void SwypeCodeDetectorDelta2::SetSwypeCode(SwypeCode &code) {
    if (code.IsEmpty()) {
        Reset(true);
        _useSwypeCode = false;
    } else {
        SwypeCodeDetector::SetSwypeCode(code);
        SetCurrentStep(SwypeStep{0, code.DirectionAt(0), code.TimeToInput()}, 0);
        _useSwypeCode = true;
    }
}

void SwypeCodeDetectorDelta2::Start(uint startTimestamp) {
    if (_useSwypeCode) {
        SetSwypeCode(_code);
    }
    SwypeCodeDetector::Start(startTimestamp);
}

//
// Created by babay on 29.07.2019.
//

#include <swype/SwypeCodeDetectorDelta2.h>
#include <swype/DetectorState.h>

void SwypeCodeDetectorDelta2::SetNextStep(SwypeStep step) {
    _nextStep = step;
    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("set swype next step. num: %d, dir: %d, dur: %d",
                    step.number, step.direction, step.maxDurationMs);
    }
}

void SwypeCodeDetectorDelta2::NextFrame(VectorExplained shift) {
    _currentTimestamp = shift._timestamp;
    if (shift._timestamp < _startTimestamp) {
    } else if (shift._timestamp > _maxTimestamp) {
        _status = -2;
    } else if (shift._mod <= 0) { // generally == 0
        _status = 0;
    } else {
        _stepDetector.Add(shift);
        _status = _stepDetector.CheckState(false);
        if (_status == 1 && _useSwypeCode && _currentStep.number + 1 >= _code._length) {
            _status = 2;
        }
    }
}

unsigned int SwypeCodeDetectorDelta2::TimeToFail() {
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
        SetCurrentStep(SwypeStep{num, _code._directions[num], duration}, _currentTimestamp);
    } else {
        SetCurrentStep(_nextStep, _currentTimestamp);
        _nextStep = {0, 0, 0};
    }
}

void SwypeCodeDetectorDelta2::SetSwypeCode(SwypeCode &code) {
    if (code._length > 0) {
        SwypeCodeDetector::SetSwypeCode(code);
        SetCurrentStep(SwypeStep{0, code._directions[0], code.TimeToInput()}, 0);
        _useSwypeCode = true;
    } else {
        Reset(true);
        _useSwypeCode = false;
    }
}

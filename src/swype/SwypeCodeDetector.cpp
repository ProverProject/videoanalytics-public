//
// Created by babay on 20.06.2018.
//

#include "swype/swype_detect.h"
#include "swype/SwypeCodeDetector.h"


unsigned int SwypeCodeDetector::counter = 0;

void SwypeCodeDetector::FillResult(int &status, int &index, int &x, int &y, int &message, int &debug) const {
    x = (int) (_resultX * 1024);
    y = (int) (_resultY * 1024);

    debug = (int) (_stepDetector._current._defectX * 1024);
    debug = debug << 16;
    debug += (int) (_stepDetector._current._defectY * 1024);

    FillResult(status, index, message);
}


void SwypeCodeDetector::FillResult(int &status, int &index, int &message) const {
    index = _currentStep + 1;

    switch (SwypeCodeDetector::_status){
        case -1:
            status = DetectorState::WaitingForCircle;
            message = Message::SwypeFailOutOfBounds;
            break;

        case -2:
            status = DetectorState::WaitingForCircle;
            message = Message::SwypeFailTimeout;
            break;

        case 1:
            status = DetectorState::SwypeCodeDone;
            message = Message::None;
            break;

        case 2:
            status = DetectorState::WaitingToStartSwypeCode;
            message = Message::None;
            break;

        case 0:
        default:
            status = DetectorState::DetectingSwypeCode;
            message = Message::None;
            break;
    }
}


void SwypeCodeDetector::GetCurrentVector(float &x, float &y, float &dx, float &dy) {
    x = static_cast<float>(_resultX);
    y = static_cast<float>(_resultY);
    dx = _stepDetector._current._defectX;
    dy = _stepDetector._current._defectY;
}

SwypeCodeDetector::SwypeCodeDetector(SwipeCode &code, DetectorParameters parameters,
                                     unsigned int timestamp)
        : _code(code),
          _relaxed(parameters._relaxed),
          _id(++counter),
          _stepDetector(_id) {
    _stepDetector.Configure(parameters);
    _stepDetector.SetDirection(_code._directions[0]);
    _startTimestamp = timestamp + _code.PauseBeforeEnterCode();
    _maxTimestamp = _startTimestamp + _code.TimeToInput();
}

void SwypeCodeDetector::Init(SwipeCode &code, DetectorParameters parameters,
                             unsigned int timestamp) {
    _code = code;
    _relaxed = parameters._relaxed;
    _stepDetector.Configure(parameters);
    _stepDetector.SetDirection(_code._directions[0]);
    _currentStep = 0;
    _status = 2;
    _startTimestamp = timestamp + _code.PauseBeforeEnterCode();
    _maxTimestamp = _startTimestamp + _code.TimeToInput();
    _resultX = 0;
    _resultY = 0;
}

void SwypeCodeDetector::SetInstantStart() {
    uint pauseBeforeStart = _code.PauseBeforeEnterCode();
    _startTimestamp -= pauseBeforeStart;
    _maxTimestamp -= pauseBeforeStart;
}
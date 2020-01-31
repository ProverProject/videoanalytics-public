//
// Created by babay on 20.06.2018.
//

#include "swype/SwypeCodeDetector.h"

#include "swype/swype_detect.h"


unsigned int SwypeCodeDetector::_counter = 0;

void SwypeCodeDetector::FillResult(int &status, int &index, int &x, int &y, int &message,
                                   int &debug) const {
    x = (int) (_stepDetector.GetCurrent().X() * 1024);
    y = (int) (_stepDetector.GetCurrent().Y() * 1024);

    debug = (int) (_stepDetector.GetCurrent().DefectX() * 1024);
    debug = debug << 16;
    debug += (int) (_stepDetector.GetCurrent().DefectY() * 1024);

    FillResult(status, index, message);
}

void SwypeCodeDetector::FillResult(int &status, int &index, int &message) const {
    index = _currentStep.number + 1;

    switch (_status) {
        case -1:
            status = DetectorState::WaitingForCircle;
            message = Message::SwypeFailOutOfBounds;
            break;

        case -2:
            status = DetectorState::WaitingForCircle;
            message = Message::SwypeFailTimeout;
            break;

        case 1:
            ++index;
            status = DetectorState::DetectingSwypeCode;
            message = Message::SwypeStepFinished;
            break;

        case 2:
            ++index;
            status = DetectorState::SwypeCodeDone;
            message = Message::None;
            break;

        case 3:
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

void SwypeCodeDetector::GetCurrentVector(float *point, float *defect) {
    point[0] = static_cast<float>(_stepDetector.GetCurrent().X());
    point[1] = static_cast<float>(_stepDetector.GetCurrent().Y());
    if (defect != nullptr) {
        defect[0] = _stepDetector.GetCurrent().DefectX();
        defect[1] = _stepDetector.GetCurrent().DefectY();
    }
}


void SwypeCodeDetector::Init(DetectorParameters parameters) {
    _stepDetector.Configure(parameters);
}

void
SwypeCodeDetector::Init(SwypeCode &code, DetectorParameters parameters, unsigned int timestamp) {
    _code = code;
    _relaxed = parameters.IsRelaxed();
    _stepDetector.Configure(parameters);

    SwypeStep step{0, code.DirectionAt(0), _code.TimeToInput()};
    _startTimestamp = timestamp + _code.PauseBeforeEnterCode();
    SetCurrentStep(step, timestamp + _code.PauseBeforeEnterCode());
    _status = 3;
}

void SwypeCodeDetector::SetCurrentStep(SwypeStep step, unsigned int firstFrameTimestamp) {
    _currentStep = step;
    _stepDetector.SetDirection(step.direction);
    _maxTimestamp = firstFrameTimestamp + step.maxDurationMs;
    if (logLevel & LOG_GENERAL_DETECTION) {
        LOGI_NATIVE("set swype current step. num: %d, dir: %d, dur: %d; maxTimestamp: %d",
                    _currentStep.number, _currentStep.direction, _currentStep.maxDurationMs,
                    _maxTimestamp);
    }
}

void SwypeCodeDetector::AdvanceSwypeStep() {
    _currentStep.number++;
    _currentStep.direction = _code.DirectionAt(_currentStep.number);
    _currentStep.maxDurationMs = _maxTimestamp - _currentTimestamp;
#ifdef  RESET_SUM_AT_SWYPE_POINT
        _stepDetector.SetDirection(_currentStep.direction);
#else
        _stepDetector.AdvanceDirection(_currentStep.direction);
#endif
    _status = 0;
}

bool SwypeCodeDetector::IsInitialised() const {
    return _currentStep.maxDurationMs > 0;
}

void SwypeCodeDetector::Reset(bool resetSwypeCode) {
    _currentStep = {0,0,0};
    _stepDetector.Reset();
    if (resetSwypeCode) {
        _code.Clear();
    }
}

void SwypeCodeDetector::SetSwypeCode(const SwypeCode &code) {
    _code = code;
}

void SwypeCodeDetector::Start(uint startTimestamp) {
    _startTimestamp = startTimestamp;
    _maxTimestamp = startTimestamp + _currentStep.maxDurationMs;
    _status = 3;
    _stepDetector.Reset();
    _stepDetector.SetDirection(_currentStep.direction);
}



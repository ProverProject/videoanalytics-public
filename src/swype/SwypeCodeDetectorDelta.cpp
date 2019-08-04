//
// Created by babay on 21.06.2018.
//

#include <swype/SwypeCodeDetectorDelta.h>

/*
void SwypeCodeDetectorDelta::NextFrame(cv::Mat &frame_i, uint timestamp) {
    if (timestamp < _startTimestamp) {
        _shiftDetector.SetBaseFrame(frame_i);
    } else {
        int debugX, debugY;
        VectorExplained shift = _shiftDetector.ShiftToPrevFrame(frame_i, timestamp, debugX, debugY);
        if (timestamp > _maxTimestamp) {
            _status = -2;
        } else if (shift._mod <= 0) { // generally == 0
            _status = 0;
        } else {
            _stepDetector.Add(shift);
            _status = _stepDetector.CheckState(_relaxed);
            if (_status == 1) {
                if (++_currentStep >= _code._length) {
                    _stepDetector.FinishStep();
                } else {
                    if (RESET_SUM_AT_SWYPE_POINT)
                        _stepDetector.SetDirection(_code._directions[_currentStep]);
                    else
                        _stepDetector.AdvanceDirection(_code._directions[_currentStep]);
                    _status = 0;
                }
            }
        }
    }
}*/

void SwypeCodeDetectorDelta::NextFrame(VectorExplained shift) {
    _currentTimestamp = shift._timestamp;
    if (shift._timestamp < _startTimestamp) {
    } else if (shift._timestamp > _maxTimestamp) {
        _status = -2;
    } else if (shift._mod <= 0) { // generally == 0
        _status = 0;
    } else {
        _stepDetector.Add(shift);
        _status = _stepDetector.CheckState(_relaxed);
        if (_status == 1 && _currentStep.number + 1 >= _code._length) {
            _status = 2;
        }
    }
}

void SwypeCodeDetectorDelta::AdvanceStep() {
    if (_status == 2) {
        _stepDetector.FinishStep();
    } else {
        AdvanceSwypeStep();
    }
}

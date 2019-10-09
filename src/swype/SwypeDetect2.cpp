//
// Created by babay on 29.07.2019.
//

#include "swype/SwypeDetect2.h"

void SwypeDetect2::init(double sourceAspectRatio, int detectorWidth, int detectorHeight) {
    SwypeDetectorBase::init(sourceAspectRatio, detectorWidth, detectorHeight);
    _swypeDetector.Init(_detectorParameters);
    _frameWidth = detectorWidth;
    _frameHeight = detectorHeight;
}

void SwypeDetect2::Reset(bool resetSwypeCode) {
    _swypeDetector.Reset(resetSwypeCode);
    if (resetSwypeCode) {
        _swypeCodeSet = false;
        _state = DetectorState::WaitingForCode;
    } else {
        _state = DetectorState::WaitingForCircle;
    }
}

void SwypeDetect2::SetFirstStep(char direction, unsigned int maxDuration) {
    _swypeDetector.Reset(true);
    AddNextStep(direction, maxDuration);
}

void SwypeDetect2::AddNextStep(char direction, unsigned int maxDuration) {
    _swypeCodeSet = true;
    if (_swypeDetector.GetCurrentStep().direction == 0) {
        _swypeDetector.SetCurrentStep(SwypeStep{0, direction, maxDuration}, 0);
        _swypeCodeSet = true;
        _state = DetectorState::WaitingForCircle;
    } else {
        unsigned int num = _swypeDetector.GetCurrentStep().number;
        _swypeDetector.SetNextStep(SwypeStep{num + 1, direction, maxDuration});
    }
}

void SwypeDetect2::ProcessMat(const cv::Mat &frame, uint timestamp, int &state, int &index,
                              int &message, float *point, float *shift, float *defect,
                              float *circleCoordinates, int circleCoordinatesLength,
                              int &actualCircleCoordinates) {
    message = Message::None;
    if (shouldIgnoreFrame(frame, state, message)) {
        fillEmptyResponse(point, shift, defect, actualCircleCoordinates);
        //_shiftDetector.SetPrevFrame(frame);
        if (_state == DetectorState::WaitingToStartSwypeCode
            || _state == DetectorState::DetectingSwypeCode) {
            VectorExplained windowedShift(0, 0, timestamp);
            _swypeDetector.NextFrame(windowedShift);
            int msgTemp;
            _swypeDetector.FillResult(_state, index, msgTemp);
            if (msgTemp != 0)
                message = msgTemp;
            state = _state;
        }
        return;
    }

    VectorExplained windowedShift = _shiftDetector.ShiftToPrevFrame(frame, timestamp);
    if (shift != nullptr) {
        windowedShift.toFloatArray(shift);
    }

    if (_state == DetectorState::WaitingForCircle) {
        bool gotCircle = SwypeDetectorBase::DetectCircle(windowedShift,
                                                         circleCoordinates, circleCoordinatesLength,
                                                         actualCircleCoordinates, message);
        if (gotCircle && _swypeDetector.IsInitialised()) {
            _swypeDetector.Start(timestamp + PAUSE_TO_ST3);
            _circleDetector.Clear();
            _state = DetectorState::WaitingToStartSwypeCode;
        }
    } else {
        actualCircleCoordinates = 0;
    }

    if (_state <= DetectorState::WaitingForCircle) {
        index = 1;
        state = _state;
        point[0] = 0;
        point[1] = 0;
        defect[0] = 0;
        defect[1] = 0;
    } else {
        _swypeDetector.NextFrame(windowedShift);
        _swypeDetector.FillResult(_state, index, message);
        _swypeDetector.GetCurrentVector(point, defect);
        if (_swypeDetector.IsStepFinished())
            _swypeDetector.AdvanceStep();
        state = _state;
    }
    if (logLevel & LOG_GENERAL_DETECTION && point != nullptr) {
        LOGI_NATIVE("state: %d, index: %d, msg: %d, (%.2f, %.2f)",
                    state, index, message, point[0], point[1]);
    }
}

unsigned int SwypeDetect2::TimeToFailMs() const {
    if (_state == DetectorState::DetectingSwypeCode)
        return _swypeDetector.TimeToFail();
    else
        return 0;
}

void SwypeDetect2::ProcessFrame(const unsigned char *frame_i, uint timestamp, int &state,
                                int &index, int &message, float *point, float *shift,
                                float *defect, float *circleCoordinates,
                                int circleCoordinatesLength, int &actualCircleCoordinates) {
    cv::Mat frame(_frameHeight, _frameWidth, CV_8UC1, (uchar *) frame_i);
    ProcessMat(frame, timestamp, state, index, message, point, shift, defect,
               circleCoordinates, circleCoordinatesLength, actualCircleCoordinates);
}

void SwypeDetect2::SetSwypeCode(SwypeCode &code) {
    _swypeDetector.SetSwypeCode(code);
    _state = DetectorState::WaitingForCircle;
    _swypeCodeSet = true;
}


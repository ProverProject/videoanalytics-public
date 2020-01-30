//
// Created by babay on 29.07.2019.
//

#include "swype/PhaseCorrelatePeaks.h"
#include "swype/SwypeDetect2.h"

void SwypeDetect2::init(double sourceAspectRatio, int detectorWidth, int detectorHeight) {
    SwypeDetectorBase::init(sourceAspectRatio, detectorWidth, detectorHeight);
    _swypeDetector.Init(_detectorParameters);
    _frameWidth = detectorWidth;
    _frameHeight = detectorHeight;
}

void SwypeDetect2::Reset(bool resetSwypeCode) {
    _swypeDetector.Reset(resetSwypeCode);
    _circleDetector.Reset();
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

void SwypeDetect2::ProcessMat(const cv::Mat &frame, uint timestamp, DetectionResults &result) {
    if (shouldIgnoreFrame(frame, result)) {
        OnIgnoringFrame(timestamp, result);
        return;
    }

    VectorExplained shift = _shiftDetector.ShiftToPrevFrame(frame, timestamp, &result._peaks,
                                                            result._correlatedFrame);

    if (result._peaks.IsPhaseCorrelateBad()) {
        result._message = Message::BadPicture;
        OnIgnoringFrame(timestamp, result);
        if (logLevel & LOG_GENERAL_DETECTION) {
            LOGI_NATIVE(
                    "PhaseCorrelateBad: %d, index: %d, msg: %d, phWeigths: %.1f, peaks: %.1f, ptc: %.1f, p1: (%.1f,%.1f) p2: (%.1f,%.1f)",
                    result._state, result._index, result._message,
                    result._peaks.WeightedCentroidRatio(),
                    result._peaks.PeakRatio(),
                    result._peaks.PeakToCentroid(),
                    result._peaks.getPeak().x, result._peaks.getPeak().y,
                    result._peaks.getSecondPeak().x, result._peaks.getSecondPeak().y
            );
        }
        return;
    }

    result.SetShift(shift);

    if (_state == DetectorState::WaitingForCircle) {
        bool gotCircle = SwypeDetectorBase::DetectCircle(shift, result);
        if (gotCircle && _swypeDetector.IsInitialised()) {
            _swypeDetector.Start(timestamp + PAUSE_TO_ST3);
            _circleDetector.Clear();
            _state = DetectorState::WaitingToStartSwypeCode;
        }
    }

    if (_state <= DetectorState::WaitingForCircle) {
        result._index = 1;
        result._state = _state;
    } else {
        _swypeDetector.NextFrame(shift);
        _swypeDetector.FillResult(_state, result._index, result._message);
        _swypeDetector.GetCurrentVector(result._point, result._defect);
        if (_swypeDetector.IsStepFinished())
            _swypeDetector.AdvanceStep();
        result._state = _state;
    }
    result._timeToFailMs = TimeToFailMs();
    if (logLevel & LOG_GENERAL_DETECTION && result._point != nullptr && shift.Mod() > 0.1) {
        LOGI_NATIVE(
                "state: %d, index: %d, msg: %d, phWeigths: %.2f, peaks: %.2f, ptc: %.2f, (%.2f, %.2f)(%.2f, %.2f)",
                result._state, result._index, result._message,
                result._peaks.WeightedCentroidRatio(),
                result._peaks.PeakRatio(),
                result._peaks.PeakToCentroid(),
                result._shift[0], result._shift[1],
                result._point[0], result._point[1]);
    }
}

unsigned int SwypeDetect2::TimeToFailMs() const {
    if (_state == DetectorState::DetectingSwypeCode)
        return _swypeDetector.TimeToFail();
    else
        return 0;
}

void
SwypeDetect2::ProcessFrame(const unsigned char *frame_i, uint timestamp, DetectionResults &result) {
    cv::Mat frame(_frameHeight, _frameWidth, CV_8UC1, (uchar *) frame_i);
    ProcessMat(frame, timestamp, result);
}

void SwypeDetect2::SetSwypeCode(SwypeCode &code) {
    _swypeDetector.SetSwypeCode(code);
    _state = DetectorState::WaitingForCircle;
    _swypeCodeSet = true;
}

void SwypeDetect2::OnIgnoringFrame(uint timestamp, DetectionResults &result) {
    result._shift[0] = 0.0f;
    result._shift[1] = 0.0f;
    if (!_swypeCodeSet) {
        result._state = DetectorState::WaitingForCode;
        return;
    }

    switch (_state) {
        case DetectorState::WaitingToStartSwypeCode:
        case DetectorState::DetectingSwypeCode:
#ifdef RESET_DETECTOR_ON_BAD_PICTURE
            _state = result._state = DetectorState::WaitingForCircle;
#else
        {
            VectorExplained windowedShift(0, 0, timestamp);
            _swypeDetector.NextFrame(windowedShift);
            int msgTemp;
            _swypeDetector.FillResult(_state, result._index, msgTemp);
            _swypeDetector.GetCurrentVector(result._point, result._defect);
            if (msgTemp != 0)
                result._message = msgTemp;
            result._state = _state;
            result._timeToFailMs = TimeToFailMs();
        }
#endif
            break;

        case DetectorState::WaitingForCode:
        case DetectorState::WaitingForCircle:
        case DetectorState::SwypeCodeDone:
            result._state = _state;
            break;
    }
}


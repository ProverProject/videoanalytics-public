
#include "swype/swype_detect.h"

#include "swype/SwypeCodeDetector.h"
#include "swype/SwypeDetectorBase.h"

using namespace cv;
using namespace std;

SwypeDetect::SwypeDetect() : SwypeDetectorBase() // initialization
{
    ocl::setUseOpenCL(true);
    _state = 0;
}

SwypeDetect::~SwypeDetect() {
    ocl::setUseOpenCL(false);
}

void SwypeDetect::setSwype(const string &swype) {
    _swypeCode = SwypeCode(swype);
    _swypeCodeSet = !_swypeCode.IsEmpty();
    _detectorParameters.ForCode(_swypeCode);
    _state = DetectorState::WaitingForCircle;
}

void
SwypeDetect::processMat(const Mat &frame, const uint timestamp, int &state, int &index, int &x,
                        int &y,
                        int &message, int &debug) {
    message = Message::None;
    if (SwypeDetectorBase::shouldIgnoreFrame(frame, state, message)) {
        return;
    }

    VectorExplained windowedShift = _shiftDetector.ShiftToPrevFrame(frame, timestamp);

    if (_state == DetectorState::WaitingForCircle) {
        DetectCircle(windowedShift, timestamp);
    }

    if (_state <= DetectorState::WaitingForCircle) {
        index = 1;
        state = _state;
    } else {
        _detector.NextFrame(windowedShift);
        _detector.FillResult(_state, index, x, y, message, debug);
        if (_detector.IsStepFinished())
            _detector.AdvanceStep();
        state = _state;
    }
}

void
SwypeDetect::processFrameExt(const unsigned char *frame_i, int width_i, int height_i,
                             uint timestamp,
                             int &state, int &index, int &message, float *resultCoordinates,
                             int resultCoordinatesLength, int &actualResultCoordinates) {

    Mat frame(height_i, width_i, CV_8UC1, (uchar *) frame_i);
    CV_Assert(_detectorParameters.IsOfSize(width_i, height_i));
    processMatExt(frame, timestamp, state, index, message,
                  resultCoordinates, resultCoordinatesLength, actualResultCoordinates);
}

void
SwypeDetect::processMatExt(const cv::Mat &frame, uint timestamp, int &state, int &index,
                           int &message,
                           float *resultCoordinates, int resultCoordinatesLength,
                           int &actualResultCoordinates) {

    float *point = resultCoordinates;
    float *shift = resultCoordinatesLength < 4 ? nullptr : resultCoordinates + 2;
    float *defect = resultCoordinatesLength < 6 ? nullptr : resultCoordinates + 4;
    float *circle = resultCoordinatesLength < 8 ? nullptr : resultCoordinates + 6;
    int circleCoordinates = resultCoordinatesLength < 8 ? 0 : resultCoordinatesLength - 6;
    int actualCircleCoordinates = 0;

    processMatExt(frame, timestamp, state, index, message,
                  point, shift, defect, circle, circleCoordinates, actualCircleCoordinates);

    if (resultCoordinatesLength <= 6) {
        actualResultCoordinates = resultCoordinatesLength;
    } else {
        actualResultCoordinates = actualCircleCoordinates + 6;
    }

}

void
SwypeDetect::DetectCircle(const VectorExplained &windowedShift, uint timestamp) {
    if (windowedShift.Mod() > 0) {
        _circleDetector.AddShift(windowedShift);
        if (_circleDetector.IsCircle()) {
            _detector.Init(_swypeCode, _detectorParameters, timestamp);
            _circleDetector.Clear();
            _state = DetectorState::WaitingToStartSwypeCode;
        }
    }
}

void SwypeDetect::processMatExt(const cv::Mat &frame, uint timestamp, int &state, int &index,
                                int &message, float *point, float *shift, float *defect,
                                float *circleCoordinates, int circleCoordinatesLength,
                                int &actualCircleCoordinates) {

    message = Message::None;
    if (shouldIgnoreFrame(frame, state, message)) {
        fillEmptyResponse(point, shift, defect, actualCircleCoordinates);
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
        if (gotCircle) {
            _detector.Init(_swypeCode, _detectorParameters, timestamp);
            _circleDetector.Clear();
            _state = DetectorState::WaitingToStartSwypeCode;
        }
    }

    if (_state <= DetectorState::WaitingForCircle) {
        index = 1;
        state = _state;
    } else {
        _detector.NextFrame(windowedShift);
        _detector.FillResult(_state, index, message);
        _detector.GetCurrentVector(point, defect);
        if (_detector.IsStepFinished()) {
            _detector.AdvanceStep();
        }
        state = _state;
    }
}


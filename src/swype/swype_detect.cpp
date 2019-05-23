#include "swype/swype_detect.h"
#include "swype/SwypeCodeDetector.h"

using namespace cv;
using namespace std;

SwypeDetect::SwypeDetect() // initialization
{
    ocl::setUseOpenCL(true);
    _state = 0;
}

SwypeDetect::~SwypeDetect() {
    ocl::setUseOpenCL(false);
}

void SwypeDetect::init(double sourceAspectRatio, int detectorWidth, int detectorHeight) {
    init(sourceAspectRatio, detectorWidth, detectorHeight, true);
}

void
SwypeDetect::init(double sourceAspectRatio, int detectorWidth, int detectorHeight, bool relaxed) {
    _shiftDetector.SetDetectorSize(detectorWidth, detectorHeight, sourceAspectRatio);
    //_debugComparer.SetShiftDetector(_shiftDetector);
    setRelaxed(relaxed);
}

bool SwypeDetect::useOpenCL() {
    return ocl::useOpenCL();
}

void SwypeDetect::setRelaxed(bool relaxed) {
    _shiftDetector.SetRelativeDefect(relaxed ? DEFECT : DEFECT_CLIENT);
    _circleDetector.SetRelaxed(relaxed);
    _detectorParameters.SetRelaxed(relaxed);

    if (relaxed)
        _histogtam.Configure(MIN_BRIGHTNESS_SERVER, MIN_CONTRAST_SERVER);
    else
        _histogtam.Configure(MIN_BRIGHTNESS_CLIENT, MIN_CONTRAST_CLIENT);
}

void SwypeDetect::setSwype(string swype) {
    _swypeCode.Init(swype);
    _detectorParameters.ForCode(_swypeCode);
    _state = DetectorState::WaitingForCircle;
}

void
SwypeDetect::processFrame(const unsigned char *frame_i, int width_i, int height_i, uint timestamp,
                          int &state, int &index, int &x, int &y, int &message, int &debug) {
    Mat frame(height_i, width_i, CV_8UC1, (uchar *) frame_i);
    _shiftDetector.UpdateDetectorSize(width_i, height_i);
    processMat(frame, timestamp, state, index, x, y, message, debug);
}

void
SwypeDetect::processMat(const Mat &frame, const uint timestamp, int &state, int &index, int &x,
                        int &y,
                        int &message, int &debug) {
    message = Message::None;
    if (shouldIgnoreFrame(frame, state, message)) {
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
        state = _state;
    }
}

void
SwypeDetect::processFrameExt(const unsigned char *frame_i, int width_i, int height_i,
                             uint timestamp,
                             int &state, int &index, int &message, float *resultCoordinates,
                             int resultCoordinatesLength, int &actualResultCoordinates) {

    Mat frame(height_i, width_i, CV_8UC1, (uchar *) frame_i);
    _shiftDetector.UpdateDetectorSize(width_i, height_i);
    processMatExt(frame, timestamp, state, index, message,
                  resultCoordinates, resultCoordinatesLength, actualResultCoordinates);
}

void
SwypeDetect::processMatExt(const cv::Mat &frame, uint timestamp, int &state, int &index,
                           int &message,
                           float *resultCoordinates, int resultCoordinatesLength,
                           int &actualResultCoordinates) {
    message = Message::None;
    if (shouldIgnoreFrame(frame, state, message)) {
        fillEmptyResponse(resultCoordinates, resultCoordinatesLength, actualResultCoordinates);
        return;
    }

    VectorExplained windowedShift = _shiftDetector.ShiftToPrevFrame(frame, timestamp);
    if (resultCoordinatesLength >= 4) {
        resultCoordinates[2] = static_cast<float>(windowedShift._x);
        resultCoordinates[3] = static_cast<float>(windowedShift._y);
    }

    int gotCircleCoordinates = 0;

    if (_state == DetectorState::WaitingForCircle) {
        DetectCircle(windowedShift, timestamp, resultCoordinates + 6, resultCoordinatesLength - 6,
                     gotCircleCoordinates, message);
    }

    if (_state <= DetectorState::WaitingForCircle) {
        index = 1;
        state = _state;
    } else {
        _detector.NextFrame(windowedShift);
        _detector.FillResult(_state, index, message);
        if (resultCoordinatesLength >= 6) {
            _detector.GetCurrentVector(resultCoordinates[0], resultCoordinates[1],
                                       resultCoordinates[4], resultCoordinates[5]);
        } else {
            float t1, t2;
            _detector.GetCurrentVector(resultCoordinates[0], resultCoordinates[1], t1, t2);
        }
        state = _state;
    }
    actualResultCoordinates =
            resultCoordinatesLength <= 6 ? resultCoordinatesLength : gotCircleCoordinates + 6;
}

void
SwypeDetect::DetectCircle(VectorExplained windowedShift, uint timestamp) {
    if (windowedShift._mod > 0) {
        _circleDetector.AddShift(windowedShift);
        if (_circleDetector.IsCircle()) {
            _detector.Init(_swypeCode, _detectorParameters, timestamp);
            _circleDetector.Clear();
            _state = DetectorState::WaitingToStartSwypeCode;
        }
    }
}

inline void
SwypeDetect::DetectCircle(VectorExplained windowedShift, uint timestamp, float *resultCoordinates,
                          int resultCoordinatesLength, int &gotCircleCoordinates, int &message) {
    if (windowedShift._mod > 0) {
        _circleDetector.AddShift(windowedShift);
        unsigned int checkCircleResult;
        if (resultCoordinatesLength >= 10) {
            checkCircleResult = _circleDetector.CheckCircle(resultCoordinates,
                                                            resultCoordinatesLength,
                                                            gotCircleCoordinates);
        } else {
            int tmp;
            checkCircleResult = _circleDetector.CheckCircle(tmp);
        }
        switch (checkCircleResult) {
            case CircleDetector::GotCircle:
                _detector.Init(_swypeCode, _detectorParameters, timestamp);
                _circleDetector.Clear();
                _state = DetectorState::WaitingToStartSwypeCode;
                break;

            case CircleDetector::AreaTooSmall:
            case CircleDetector::CurveNotRoundEnough:
                message = checkCircleResult << 1;
                break;

            default:
            case CircleDetector::NoCircle:
                break;
        }
    }
}

inline bool
SwypeDetect::shouldIgnoreFrame(const cv::Mat &frame, int &state, int &message) {
    if (_state == DetectorState::SwypeCodeDone) {
        state = _state;
        return true;
    }

    if (_swypeCode.empty()) {
        state = DetectorState::WaitingForCode;
        return true;
    }

    _histogtam.Fill(frame);
    if (_histogtam.IsLuminanceLow() || _histogtam.IsContrastLow()) {
        state = _state;
        message = Message::LuminanceLow;
        return true;
    }

    return false;
}

void SwypeDetect::fillEmptyResponse(float *resultCoordinates,
                                    int resultCoordinatesLength, int &actualResultCoordinates) {
    resultCoordinates[0] = 0;
    resultCoordinates[1] = 0;
    if (resultCoordinatesLength >= 4) {
        resultCoordinates[2] = 0;
        resultCoordinates[3] = 0;
        if (resultCoordinatesLength >= 6) {
            resultCoordinates[4] = 0;
            resultCoordinates[5] = 0;
            actualResultCoordinates = 6;
        } else {
            actualResultCoordinates = 4;
        }
    } else {
        actualResultCoordinates = 2;
    }
}

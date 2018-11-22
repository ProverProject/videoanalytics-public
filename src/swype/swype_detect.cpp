#include "swype/swype_detect.h"
#include "swype/SwypeCodeDetector.h"

using namespace cv;
using namespace std;

SwypeDetect::SwypeDetect() // initialization
{
    ocl::setUseOpenCL(true);
    S = 0;
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
    S = DetectorState::WaitingForCircle;
}

void
SwypeDetect::processFrame(const unsigned char *frame_i, int width_i, int height_i, uint timestamp,
                          int &state, int &index, int &x, int &y, int &message, int &debug) {
    Mat frame(height_i, width_i, CV_8UC1, (uchar *) frame_i);
    _shiftDetector.UpdateDetectorSize(width_i, height_i);
    processMat(frame, timestamp, state, index, x, y, message, debug);
}


void
SwypeDetect::processMat(const Mat &frame, uint timestamp, int &state, int &index, int &x, int &y,
                        int &message, int &debug) {
    message = 0;
    if (S == DetectorState::SwypeCodeDone) {
        x = 0;
        y = 0;
        state = S;
        return;
    }

    if (_swypeCode.empty()) {
        x = 0;
        y = 0;
        state = DetectorState::WaitingForCode;
        return;
    }

    _histogtam.Fill(frame);
    if (_histogtam.IsLuminanceLow() || _histogtam.IsContrastLow()) {
        state = S;
        message = 1;
        return;
    }

    int debugX, debugY;
    VectorExplained windowedShift = _shiftDetector.ShiftToPrevFrame(frame, timestamp, debugX,
                                                                    debugY);

    if (S == DetectorState::WaitingForCircle) {
        DetectCircle(windowedShift, frame, timestamp);
    }

    if (S <= DetectorState::WaitingForCircle) {
        index = 1;
        state = S;
    } else {
        _detector.NextFrame(windowedShift);
        _detector.FillResult(S, index, x, y, message, debug);
        state = S;
    }
}

void
SwypeDetect::DetectCircle(VectorExplained windowedShift, const cv::Mat &frame, uint timestamp) {
    if (windowedShift._mod > 0) {
        _circleDetector.AddShift(windowedShift);
        if (_circleDetector.IsCircle()) {
            _detector.Init(_swypeCode, _detectorParameters, timestamp);
            _circleDetector.Clear();
            S = DetectorState::WaitingToStartSwypeCode;
        }
    }
}

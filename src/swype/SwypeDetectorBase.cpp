//
// Created by babay on 29.07.2019.
//

#include "swype/SwypeDetectorBase.h"

#include "swype/SwypeCodeDetector.h"
#include "swype/swype_detect.h"

SwypeDetectorBase::SwypeDetectorBase() : _state(0), _swypeCodeSet(false) {
    cv::ocl::setUseOpenCL(true);
}

SwypeDetectorBase::~SwypeDetectorBase() {
    cv::ocl::setUseOpenCL(false);
}

bool SwypeDetectorBase::useOpenCL() {
    return cv::ocl::useOpenCL();
}

void SwypeDetectorBase::init(double sourceAspectRatio, int detectorWidth, int detectorHeight) {
    _detectorParameters.SetRelaxed(false);
    _detectorParameters.SetDetectorSize(detectorWidth, detectorHeight, sourceAspectRatio);
    _detectorParameters.SetDetectorDetect(DEFECT_CLIENT);
    _shiftDetector.Configure(_detectorParameters);
    _circleDetector.SetRelaxed(false);

    _histogtam.Configure(MIN_BRIGHTNESS_CLIENT, MIN_CONTRAST_CLIENT);
}

bool
SwypeDetectorBase::DetectCircle(const VectorExplained &windowedShift, float *resultCoordinates,
                                int resultCoordinatesLength, int &gotCircleCoordinates,
                                int &message) {
    DetectionResults result(nullptr, nullptr, nullptr, resultCoordinates, resultCoordinatesLength);
    bool res = DetectCircle(windowedShift, result);
    message = result._message;
    return res;
}

bool
SwypeDetectorBase::DetectCircle(const VectorExplained &windowedShift, DetectionResults &result) {
    if (windowedShift.Mod() > 0) {
        _circleDetector.AddShift(windowedShift);
        unsigned int checkCircleResult;
        if (result._circleCoordinatesLength >= 10) {
            checkCircleResult = _circleDetector.CheckCircle(result._circleCoordinates,
                                                            result._circleCoordinatesLength,
                                                            result._actualCircleCoordinates);
        } else {
            int tmp;
            float q;
            checkCircleResult = _circleDetector.CheckCircle(tmp, q);
        }
        switch (checkCircleResult) {
            case CircleDetector::GotCircle:
                return true;

            case CircleDetector::AreaTooSmall:
            case CircleDetector::CurveNotRoundEnough:
                result._message = checkCircleResult << 1;
                break;

            default:
            case CircleDetector::NoCircle:
                break;
        }
    }
    return false;
}

bool
SwypeDetectorBase::shouldIgnoreFrame(const cv::Mat &frame, int &state, int &message) {
    DetectionResults results(nullptr, nullptr, nullptr, nullptr, 0);
    bool ret = shouldIgnoreFrame(frame, results);
    state = results._state;
    message = results._message;
    return ret;
}

bool SwypeDetectorBase::shouldIgnoreFrame(const cv::Mat &frame, DetectionResults &result) {
    if (_state == DetectorState::SwypeCodeDone) {
        return true;
    }

    if (!_swypeCodeSet) {
        return true;
    }

    _histogtam.Fill(frame);
    result._luminance = _histogtam.GetAverageLuminance();
    result._contrast = _histogtam.GetContrast();

/*    bool lowLuminance;
    bool lowContrast = false;
    if ((lowLuminance = _histogtam.IsLuminanceLow())
        || (lowContrast = _histogtam.IsContrastLow())) {
        if (logLevel && LOG_VECTORS)
            LOGI_NATIVE("detector reject frame: low lum: %d, low contrast: %d", lowLuminance,
                        lowContrast);
        result._state = _state;
        result._message = BadPicture;
        return true;
    }*/

    return false;
}

void SwypeDetectorBase::fillEmptyResponse(float *point, float *shift, float *defect,
                                          int &actualCircleCoordinates) {
    if (point != nullptr) {
        point[0] = 0;
        point[1] = 0;
    }

    if (shift != nullptr) {
        shift[0] = 0;
        shift[1] = 0;
    }

    if (defect != nullptr) {
        defect[0] = 0;
        defect[1] = 0;
    }

    actualCircleCoordinates = 0;
}

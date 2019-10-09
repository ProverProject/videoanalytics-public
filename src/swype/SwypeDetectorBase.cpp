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

void SwypeDetectorBase::setRelaxed(bool relaxed) {
    _shiftDetector.SetRelativeDefect(relaxed ? DEFECT : DEFECT_CLIENT);
    _circleDetector.SetRelaxed(relaxed);
    _detectorParameters.SetRelaxed(relaxed);

    if (relaxed)
        _histogtam.Configure(MIN_BRIGHTNESS_SERVER, MIN_CONTRAST_SERVER);
    else
        _histogtam.Configure(MIN_BRIGHTNESS_CLIENT, MIN_CONTRAST_CLIENT);
}


void SwypeDetectorBase::init(double sourceAspectRatio, int detectorWidth, int detectorHeight) {
    _shiftDetector.SetDetectorSize(detectorWidth, detectorHeight, sourceAspectRatio);
    //_debugComparer.SetShiftDetector(_shiftDetector);
    setRelaxed(false);
}

bool
SwypeDetectorBase::DetectCircle(const VectorExplained &windowedShift, float *resultCoordinates,
                                int resultCoordinatesLength, int &gotCircleCoordinates,
                                int &message) {
    gotCircleCoordinates = 0;
    if (windowedShift.Mod() > 0) {
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
                return true;

            case CircleDetector::AreaTooSmall:
            case CircleDetector::CurveNotRoundEnough:
                message = checkCircleResult << 1;
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
    if (_state == DetectorState::SwypeCodeDone) {
        state = _state;
        return true;
    }

    if (!_swypeCodeSet) {
        state = DetectorState::WaitingForCode;
        return true;
    }

    _histogtam.Fill(frame);

    bool lowLuminance;
    bool lowContrast = false;
    if ((lowLuminance = _histogtam.IsLuminanceLow())
        || (lowContrast = _histogtam.IsContrastLow())) {
        if (logLevel && LOG_VECTORS)
            LOGI_NATIVE("detector reject frame: low lum: %d, low contrast: %d", lowLuminance,
                        lowContrast);
        state = _state;
        message = LuminanceLow;
        return true;
    }

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

//
// Created by babay on 30.06.2018.
//

#ifndef SWYPE_DETECTORPARAMETERS_H
#define SWYPE_DETECTORPARAMETERS_H


#include "swype/settings.h"
#include "swype/SwipeCode.h"

class DetectorParameters {
public:
    DetectorParameters() {};

    DetectorParameters(SwipeCode code) {
        ForCode(code);
    };

    void ForCode(SwipeCode code) {
        _maxSwypeCodeLength = code._length + 1;
        _minSwypeCodeLength = code._length <= 2 ? 1 : code._length - 1;
    }

    void SetRelaxed(bool _relaxed) {
        DetectorParameters::_relaxed = _relaxed;
    }

    void Log() const {
        LOGI_NATIVE(
                "DetectorParameters: v: (%.4f, %.4f), target: %.2f, relaxed: %d, codeLength: %d-%d",
                _speedMultX, _speedMultY, _targetRadius, _relaxed, _minSwypeCodeLength,
                _maxSwypeCodeLength);
    }

    void SetDetectorDetect(double _detectorDetect) {
        DetectorParameters::_detectorDetect = _detectorDetect;
    }

public:
    double _speedMultX = SWYPE_SPEED;
    double _speedMultY = SWYPE_SPEED;
    float _targetRadius = TARGET_RADIUS;
    bool _relaxed;
    unsigned int _maxSwypeCodeLength;
    unsigned int _minSwypeCodeLength = 1;
    double _detectorDetect;
};


#endif //SWYPE_DETECTORPARAMETERS_H

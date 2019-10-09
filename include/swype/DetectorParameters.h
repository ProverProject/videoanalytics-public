//
// Created by babay on 30.06.2018.
//

#ifndef SWYPE_DETECTORPARAMETERS_H
#define SWYPE_DETECTORPARAMETERS_H


#include "swype/settings.h"
#include "swype/SwypeCode.h"

class DetectorParameters {
public:
    DetectorParameters() {};

    DetectorParameters(SwypeCode code) {
        ForCode(code);
    };

    void ForCode(SwypeCode &code) {
        _maxSwypeCodeLength = code.Length() + 1;
        if (_maxSwypeCodeLength > MAX_SWYPE_LENGTH)
            _maxSwypeCodeLength = MAX_SWYPE_LENGTH;
        _minSwypeCodeLength = code.Length() <= 2 ? 1 : code.Length() - 1;
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

    unsigned int MaxSwypeCodeLength() const {
        return _maxSwypeCodeLength;
    }

    unsigned int MinSwypeCodeLength() const {
        return _minSwypeCodeLength;
    }

public:
    double _speedMultX = SWYPE_SPEED;
    double _speedMultY = SWYPE_SPEED;
    float _targetRadius = TARGET_RADIUS;
    bool _relaxed;
    double _detectorDetect;

private:
    unsigned int _maxSwypeCodeLength;
    unsigned int _minSwypeCodeLength = 1;

};


#endif //SWYPE_DETECTORPARAMETERS_H

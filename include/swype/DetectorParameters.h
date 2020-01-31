//
// Created by babay on 30.06.2018.
//

#ifndef SWYPE_DETECTORPARAMETERS_H
#define SWYPE_DETECTORPARAMETERS_H


#include "swype/settings.h"
#include "swype/SwypeCode.h"

class DetectorParameters {
public:
    DetectorParameters() :
            _speedMultX(SWYPE_SPEED),
            _speedMultY(SWYPE_SPEED),
            _targetRadius(TARGET_RADIUS),
            _relaxed(true),
            _detectorDefect(DEFECT),
            _maxSwypeCodeLength(MAX_SWYPE_LENGTH),
            _minSwypeCodeLength(1),
            _detectorWidth(1),
            _detectorHeight(1),
            _sourceAspectRatio(1) {};

    void ForCode(SwypeCode &code) {
        _maxSwypeCodeLength = code.Length() + 1;
        if (_maxSwypeCodeLength > MAX_SWYPE_LENGTH)
            _maxSwypeCodeLength = MAX_SWYPE_LENGTH;
        _minSwypeCodeLength = code.Length() <= 2 ? 1 : code.Length() - 1;
    }

    void SetRelaxed(bool relaxed) {
        _relaxed = relaxed;
    }

    void Log() const {
        LOGI_NATIVE(
                "DetectorParameters: v: (%.4f, %.4f), target: %.2f, relaxed: %d, codeLength: %d-%d",
                _speedMultX, _speedMultY, _targetRadius, _relaxed, _minSwypeCodeLength,
                _maxSwypeCodeLength);
    }

    void SetDetectorDetect(double detectorDetect) {
        _detectorDefect = detectorDetect;
    }

    unsigned int MaxSwypeCodeLength() const {
        return _maxSwypeCodeLength;
    }

    unsigned int MinSwypeCodeLength() const {
        return _minSwypeCodeLength;
    }

    double SpeedMultX() const {
        return _speedMultX;
    }

    double SpeedMultY() const {
        return _speedMultY;
    }

    float TargetRadius() const {
        return _targetRadius;
    }

    bool IsRelaxed() const {
        return _relaxed;
    }

    double DetectorDefect() const {
        return _detectorDefect;
    }

    void SetDetectorSize(int detectorWidth, int detectorHeight, double sourceAspectRatio){
        CV_Assert(detectorWidth > 0);
        CV_Assert(detectorHeight > 0);
        CV_Assert(sourceAspectRatio > 0);

        _detectorWidth = detectorWidth;
        _detectorHeight = detectorHeight;
        _sourceAspectRatio = sourceAspectRatio;
    }

    int DetectorWidth() const {
        return _detectorWidth;
    }

    int DetectorHeight() const {
        return _detectorHeight;
    }

    double SourceAspectRatio() const {
        return _sourceAspectRatio;
    }

    inline bool IsOfSize(int width, int height) const{
        return _detectorWidth == width && _detectorHeight == height;
    }

private:
    double _speedMultX;
    double _speedMultY;
    float _targetRadius;
    bool _relaxed;
    double _detectorDefect;

    unsigned int _maxSwypeCodeLength;
    unsigned int _minSwypeCodeLength;

    int _detectorWidth;
    int _detectorHeight;
    double _sourceAspectRatio;
};


#endif //SWYPE_DETECTORPARAMETERS_H

//
// Created by babay on 21.06.2018.
//

#include <swype/optimizedPhaseCorrelate.h>
#include "swype/ShiftDetector.h"
#include "swype/common.h"
#include "swype/settings.h"

ShiftDetector::ShiftDetector(const ShiftDetector &source) :
        _videoAspect(source._videoAspect), _detectorWidth(source._detectorWidth),
        _detecttorHeight(source._detecttorHeight), _xMult(source._xMult), _yMult(source._yMult),
        _relativeDefect(source._relativeDefect) {
}

void
ShiftDetector::SetDetectorSize(int detectorWidth, int detectorHeight, double sourceAspectRatio) {
    _detectorWidth = detectorWidth;
    _detecttorHeight = detectorHeight;
    _videoAspect = sourceAspectRatio > 1 ? sourceAspectRatio : 1.0 / sourceAspectRatio;
    if (detectorWidth > detectorHeight) {
        _yMult = -2.0 / detectorHeight;
        _xMult = -2.0 / detectorWidth * _videoAspect;
    } else {
        _xMult = -2.0 / detectorWidth;
        _yMult = -2.0 / detectorHeight * _videoAspect;
    }
    _hannWithBorder.release();
    _tickFrame.release();
    _tockFrame.release();
    _tickFFT.release();
    _tockFFT.release();

    if (logLevel > 0) {
        LOGI_NATIVE("SetDetectorSize (%d, %d) sourceAspect %f, -> (%f, %f)", detectorWidth,
                    detectorHeight, _videoAspect, _xMult, _yMult);
    }
}

VectorExplained
ShiftDetector::ShiftToPrevFrame(const cv::Mat &frame_i, uint timestamp) {
    if (_tickFrame.empty()) {
        frame_i.convertTo(_tickFrame, CV_64F);// converting frames to CV_64F type
        cv::UMat hann;
        createHanningWindow(hann, _tickFrame.size(), CV_64F); //  create Hanning window
        doPaddedWindow(hann, _hannWithBorder);
        doFFT(_tickFrame, _hannWithBorder, _tickFFT);

        _tickTock = false;
        return {0,0};
    }

    cv::Point2d shift;

    _tickTock = !_tickTock;
    if (_tickTock) {
        frame_i.convertTo(_tockFrame, CV_64F);// converting frames to CV_64F type
        doFFT(_tockFrame, _hannWithBorder, _tockFFT);
        shift = myPhaseCorrelatePart2(_tickFFT, _tockFFT);
    } else {
        frame_i.convertTo(_tickFrame, CV_64F);// converting frames to CV_64F type
        doFFT(_tickFrame, _hannWithBorder, _tickFFT);
        shift = myPhaseCorrelatePart2(_tockFFT, _tickFFT); // we calculate a phase offset vector
    }
    VectorExplained scaledShift(shift, _xMult, _yMult, timestamp);
    VectorExplained windowedShift = scaledShift;
    windowedShift.ApplyWindow(VECTOR_WINDOW_START, VECTOR_WINDOW_END);
    windowedShift.setRelativeDefect(_relativeDefect);

    if (logLevel & LOG_VECTORS) {
        log1(timestamp, shift, scaledShift, windowedShift);
    }

    return windowedShift;
}

/*
    clock_gettime(CLOCK_REALTIME, &ts5);

    double d1 = deltaTimeMks(ts2,ts1);
    double d2 = deltaTimeMks(ts3,ts2);
    double d3 = deltaTimeMks(ts4,ts3);
    double d4 = deltaTimeMks(ts5,ts4);
    double dAll = deltaTimeMks(ts5,ts1);
    LOGI_NATIVE("ShiftToPrevFrame, mks: %.1lf; %.1lf; %.1lf; %.1lf; %.1lf\n", d1, d2, d3, d4, dAll);*/


void ShiftDetector::SetPrevFrame(const cv::Mat &frame_i) {
    if (_tickTock) {
        frame_i.convertTo(_tockFrame, CV_64F);// converting frames to CV_64F type
    } else {
        frame_i.convertTo(_tickFrame, CV_64F);// converting frames to CV_64F type
    }
}


void ShiftDetector::SetBaseFrame(const cv::Mat &frame) {
    frame.convertTo(_tickFrame, CV_64F);// converting frames to CV_64F type
    if (_hannWithBorder.empty()) {
        createHanningWindow(_hannWithBorder, _tickFrame.size(), CV_64F);
    }
    _tickTock = false;
}

VectorExplained ShiftDetector::ShiftToBaseFrame(const cv::Mat &frame, uint timestamp) {
    frame.convertTo(_tockFrame, CV_64F);// converting frames to CV_64F type

    if (_hannWithBorder.empty()) {
        createHanningWindow(_hannWithBorder, _tockFrame.size(), CV_64F);
    }

    const cv::Point2d &shift = phaseCorrelate(_tickFrame, _tockFrame,
                                              _hannWithBorder); // we calculate a phase offset vector
    VectorExplained scaledShift(shift, _xMult, _yMult, timestamp);
    scaledShift.setRelativeDefect(_relativeDefect);

    if (logLevel & LOG_VECTORS) {
        log2(timestamp, shift, scaledShift);
    }

    return scaledShift;
}


void ShiftDetector::log1(uint timestamp, cv::Point2d &shift, VectorExplained &scaledShift,
                         VectorExplained &windowedShift) const {
    LOGI_NATIVE(
            "detector t%d shift (%+6.2f,%+6.2f), scaled |%+.4f,%+.4f|=%.4f windowed |%+.4f,%+.4f|=%.4f_%3.0f_%d",
            timestamp, shift.x, shift.y,
            scaledShift.X(), scaledShift.Y(), scaledShift.Mod(),
            windowedShift.X(), windowedShift.Y(), windowedShift.Mod(), windowedShift.Angle(),
            windowedShift.Direction());
}

void
ShiftDetector::log2(uint timestamp, const cv::Point2d &shift, VectorExplained &scaledShift) const {
    LOGI_NATIVE(
            "t%d shift (%+6.2f,%+6.2f), scaled |%+.4f,%+.4f|=%.4f_%3.0f_%d",
            timestamp, shift.x, shift.y,

            scaledShift.X(), scaledShift.Y(), scaledShift.Mod(), scaledShift.Angle(),
            scaledShift.Direction());
}

void ShiftDetector::SetRelativeDefect(double defect) {
    _relativeDefect = defect;
}

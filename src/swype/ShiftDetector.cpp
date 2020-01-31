//
// Created by babay on 21.06.2018.
//

#include "swype/ShiftDetector.h"
#include "swype/common.h"
#include "swype/settings.h"


void ShiftDetector::Configure(const DetectorParameters &parameters) {
    _relativeDefect = parameters.DetectorDefect();

    CV_Assert(parameters.DetectorWidth() > 0);
    CV_Assert(parameters.DetectorHeight() > 0);
    CV_Assert(parameters.SourceAspectRatio() > 0);

    double sourceAspectRatio = parameters.SourceAspectRatio();

    _detectorWidth = parameters.DetectorWidth();
    _detectorHeight = parameters.DetectorHeight();
    _videoAspect = sourceAspectRatio > 1 ? sourceAspectRatio : 1.0 / sourceAspectRatio;
    if (_detectorWidth > _detectorHeight) {
        _yMult = -2.0 / _detectorHeight;
        _xMult = -2.0 / _detectorWidth * _videoAspect;
    } else {
        _xMult = -2.0 / _detectorWidth;
        _yMult = -2.0 / _detectorHeight * _videoAspect;
    }
    _hannWithBorder.release();
    _tickFrame.release();
    _tockFrame.release();
    _tickFFT.release();
    _tockFFT.release();

    if (logLevel > 0) {
        LOGI_NATIVE("Configure Shift detector (%d, %d) sourceAspect %f, -> (%f, %f)", _detectorWidth,
                    _detectorHeight, _videoAspect, _xMult, _yMult);
    }
}

VectorExplained
ShiftDetector::ShiftToPrevFrame(const cv::Mat &frame_i, uint timestamp,
                                PhaseCorrelatePeaks *peaks,
                                PhaseCorrelateDebugFrame *debugFrame) {
    if (_tickFrame.empty()) {
        frame_i.convertTo(_tickFrame, CV_64F);// converting frames to CV_64F type
        cv::UMat hann;
        createHanningWindow(hann, _tickFrame.size(), CV_64F); //  create Hanning window
        doPaddedWindow(hann, _hannWithBorder);
        doFFT(_tickFrame, _hannWithBorder, _tickFFT);

        _tickTock = false;
        return {0, 0};
    }

    Peak peak, peak2;

    _tickTock = !_tickTock;
    if (_tickTock) {
        frame_i.convertTo(_tockFrame, CV_64F);// converting frames to CV_64F type
        doFFT(_tockFrame, _hannWithBorder, _tockFFT);
        myPhaseCorrelatePart2(_tickFFT, _tockFFT, peak, &peak2, debugFrame);
    } else {
        frame_i.convertTo(_tickFrame, CV_64F);// converting frames to CV_64F type
        doFFT(_tickFrame, _hannWithBorder, _tickFFT);
        // we calculate a phase offset vector
        myPhaseCorrelatePart2(_tockFFT, _tickFFT, peak, &peak2, debugFrame);
    }
    VectorExplained scaledShift(peak, _xMult, _yMult, timestamp);
    VectorExplained windowedShift = scaledShift;
    windowedShift.ApplyWindow(VECTOR_WINDOW_START, VECTOR_WINDOW_END);
    windowedShift.setRelativeDefect(_relativeDefect);

    if (logLevel & LOG_VECTORS) {
        log1(timestamp, peak, scaledShift, windowedShift);
    }
    if (peaks != nullptr) {
        peaks->Set(peak, peak2);
    }

    return windowedShift;
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


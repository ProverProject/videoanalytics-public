//
// Created by babay on 21.06.2018.
//

#ifndef SWYPE_SHIFTDETECTOR_H
#define SWYPE_SHIFTDETECTOR_H


#include <opencv2/core/mat.hpp>
#include "swype/VectorExplained.h"
#include "PhaseCorrelatePeaks.h"
#include "optimizedPhaseCorrelate.h"

class ShiftDetector {
public:
    friend class DebugComparer;

    ShiftDetector() {
        LOGI_NATIVE("cv version: %s", CV_VERSION);
    };

    ShiftDetector(const ShiftDetector &source);

    void SetDetectorSize(int detectorWidth, int detectorHeight, double sourceAspectRatio);

    /**
     * detects shift;
     * @param frame_i - [in] current frame
     * @param timestamp - [in] current frame timestamp
     * @return
     */

    VectorExplained
    ShiftToPrevFrame(const cv::Mat &frame_i, uint timestamp,
                     PhaseCorrelatePeaks *peaks = nullptr,
                     PhaseCorrelateDebugFrame *debugFrame = nullptr);

    void SetPrevFrame(const cv::Mat &frame_i);

    VectorExplained ShiftToBaseFrame(const cv::Mat &frame, uint timestamp);

    void SetBaseFrame(const cv::Mat &frame);

    inline void UpdateDetectorSize(int width, int height) {
        if (_detectorWidth != width || _detecttorHeight != height) {
            SetDetectorSize(width, height, _videoAspect);
        }
    }

    void SetRelativeDefect(double defect);

    bool IsBaseFrameEmpty() const { return _tickFrame.empty(); };

    int GetWidth() const {
        return _detectorWidth;
    }

    int GetHeight() const {
        return _detecttorHeight;
    }

private:
    void log1(uint timestamp, cv::Point2d &shift, VectorExplained &scaledShift,
              VectorExplained &windowedShift) const;

    void log2(uint timestamp, const cv::Point2d &shift, VectorExplained &scaledShift) const;

    cv::UMat _tickFrame;
    cv::UMat _tockFrame;
    cv::UMat _hannWithBorder;

    cv::UMat _tickFFT;
    cv::UMat _tockFFT;
    bool _tickTock = false;

    double _videoAspect = 0.0;
    int _detectorWidth = 0;
    int _detecttorHeight = 0;
    double _xMult = 0.0;
    double _yMult = 0.0;

    double _relativeDefect;// relaxed ? DEFECT : DEFECT_CLIENT
};

#endif //SWYPE_SHIFTDETECTOR_H

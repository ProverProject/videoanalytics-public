//
// Created by babay on 21.06.2018.
//

#ifndef SWYPE_SHIFTDETECTOR_H
#define SWYPE_SHIFTDETECTOR_H


#include <opencv2/core/mat.hpp>
#include "swype/VectorExplained.h"
#include "PhaseCorrelatePeaks.h"
#include "optimizedPhaseCorrelate.h"
#include "DetectorParameters.h"

class ShiftDetector {
public:
    ShiftDetector() :
            _tickTock(false),
            _videoAspect(0.0),
            _detectorWidth(0),
            _detectorHeight(0),
            _xMult(0.0),
            _yMult(0.0),
            _relativeDefect(0) {
        //LOGI_NATIVE("cv version: %s", CV_VERSION);
    };

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

    VectorExplained ShiftToBaseFrame(const cv::Mat &frame, uint timestamp);

    void SetBaseFrame(const cv::Mat &frame);

    bool IsBaseFrameEmpty() const { return _tickFrame.empty(); };

    int GetWidth() const {
        return _detectorWidth;
    }

    int GetHeight() const {
        return _detectorHeight;
    }

    void Configure(const DetectorParameters &parameters);

private:
    void log1(uint timestamp, cv::Point2d &shift, VectorExplained &scaledShift,
              VectorExplained &windowedShift) const;

    void log2(uint timestamp, const cv::Point2d &shift, VectorExplained &scaledShift) const;

    cv::UMat _tickFrame;
    cv::UMat _tockFrame;
    cv::UMat _hannWithBorder;

    cv::UMat _tickFFT;
    cv::UMat _tockFFT;
    bool _tickTock;

    double _videoAspect;
    int _detectorWidth;
    int _detectorHeight;
    double _xMult;
    double _yMult;

    double _relativeDefect;// relaxed ? DEFECT : DEFECT_CLIENT
};

#endif //SWYPE_SHIFTDETECTOR_H

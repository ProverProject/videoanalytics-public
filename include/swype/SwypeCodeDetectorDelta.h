//
// Created by babay on 21.06.2018.
//

#ifndef SWYPE_SWYPECODEDETECTORDELTA_H
#define SWYPE_SWYPECODEDETECTORDELTA_H


#include <opencv2/core/mat.hpp>
#include "swype/ShiftDetector.h"
#include "swype/SwypeCodeDetector.h"

class SwypeCodeDetectorDelta : public SwypeCodeDetector {
public:

    SwypeCodeDetectorDelta() : SwypeCodeDetector() {};

    /*SwypeCodeDetectorDelta(SwipeCode &code, DetectorParameters detectorParameters,
                           unsigned int timestamp, const ShiftDetector &shiftDetectorSettings,
                           cv::Mat &baseFrame) : SwypeCodeDetector(code, detectorParameters, timestamp),
                                                 _shiftDetector(shiftDetectorSettings) {
        _shiftDetector.SetBaseFrame(baseFrame);
    };*/
    SwypeCodeDetectorDelta(SwipeCode &code, DetectorParameters detectorParameters,
                           unsigned int timestamp, const ShiftDetector &shiftDetectorSettings,
                           const cv::Mat &baseFrame) :
            SwypeCodeDetector(code, detectorParameters, timestamp) {};

    //void NextFrame(cv::Mat &frame_i, uint timestamp) override;
    void NextFrame(VectorExplained shift);


private:
    //ShiftDetector _shiftDetector;
};


#endif //SWYPE_SWYPECODEDETECTORDELTA_H

//
// Created by babay on 06.01.2018.
//

#ifndef PROVER_MVP_ANDROID_SWYPECODEDETECTOR_H
#define PROVER_MVP_ANDROID_SWYPECODEDETECTOR_H


#include "swype/SwypeStepDetector.h"
#include "swype/SwipeCode.h"
#include "swype/SwypeCodeDetector.h"
#include "swype/ShiftDetector.h"

class SwypeCodeDetectorBaseFrame : public SwypeCodeDetector {
public:

    SwypeCodeDetectorBaseFrame() : SwypeCodeDetector() {}

    SwypeCodeDetectorBaseFrame(SwipeCode &code, DetectorParameters parameters,
                               unsigned int timestamp, const ShiftDetector &shiftDetectorSettings,
                               cv::Mat &baseFrame) : SwypeCodeDetector(code, parameters, timestamp),
                                                     _shiftDetector(shiftDetectorSettings) {
        SetBaseFrame(baseFrame);
    };

    void NextFrame(cv::Mat &frame_i, uint timestamp);// override;

    void SetBaseFrame(cv::Mat &frame);

private:
    ShiftDetector _shiftDetector;
};

#endif //PROVER_MVP_ANDROID_SWYPECODEDETECTOR_H

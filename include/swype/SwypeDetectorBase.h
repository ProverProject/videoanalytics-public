//
// Created by babay on 29.07.2019.
//

#ifndef PROVER_SWYPEDETECTORBASE_H
#define PROVER_SWYPEDETECTORBASE_H


#include <opencv2/opencv.hpp>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include "opencv2/core/ocl.hpp"
#include "swype/VectorExplained.h"
#include "swype/SwypeStepDetector.h"
#include "swype/CircleDetector.h"
#include "swype/DetectorState.h"
#include "swype/ShiftDetector.h"
#include "swype/SwypeCodeDetectorDelta.h"
#include "swype/Histogram.h"

class SwypeDetectorBase {
protected:

public:
    SwypeDetectorBase();

    virtual ~SwypeDetectorBase();

    /**
       * check whether OpenCL is being used
       */
    bool useOpenCL();

    void setRelaxed(bool relaxed);

/**
 *
 * @param sourceAspectRatio - aspect ratio (width / height) of original video
 * @param detectorWidth - detector frame width, px
 * @param detectorHeight - detector frame height, px
 */
    virtual void init(double sourceAspectRatio, int detectorWidth, int detectorHeight);

    virtual void
    init(double sourceAspectRatio, int detectorWidth, int detectorHeight, bool relaxed);


protected:

    bool DetectCircle(VectorExplained windowedShift, uint timestamp, float *resultCoordinates,
                      int resultCoordinatesLength, int &gotCircleCoordinates, int &message);

    bool shouldIgnoreFrame(const cv::Mat &frame, int &state, int &message);

    void fillEmptyResponse(float *point, float *shift, float *defect, int &actualCircleCoordinates);

    bool _swypeCodeSet;
    int _state;
    Histogram _histogtam;
    ShiftDetector _shiftDetector;
    CircleDetector _circleDetector;
    DetectorParameters _detectorParameters;

};


#endif //PROVER_SWYPEDETECTORBASE_H

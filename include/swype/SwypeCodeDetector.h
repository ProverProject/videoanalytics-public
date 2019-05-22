//
// Created by babay on 20.06.2018.
//

#ifndef SWYPE_SWYPECODEDETECTORBASE_H
#define SWYPE_SWYPECODEDETECTORBASE_H


#include <opencv2/core/mat.hpp>
#include "swype/SwypeStepDetector.h"
#include "swype/SwipeCode.h"

class SwypeCodeDetector {
public:

    virtual ~SwypeCodeDetector() {};

    SwypeCodeDetector() : _id(++counter), _stepDetector(_id) {};

    SwypeCodeDetector(SwipeCode &code, DetectorParameters detectorParameters,
                      unsigned int timestamp);

    //virtual void NextFrame(cv::Mat &frame_i, uint timestamp) = 0;

    void FillResult(int &status, int &index, int &x, int &y, int &message, int &debug) const;

    void FillResult(int &status, int &index, int &message) const;

    /**
     * return current vector size and defect
     * @param x
     * @param y
     * @param dx
     * @param dy
     */
    void GetCurrentVector(float &x, float &y, float &dx, float &dy);

    /*
     *    1 -- swype code completed
     *    0 -- processing swype code
     *    2 -- waiting to start swype code processing
     *   -1 -- swype code failed
     *   -2 -- swype input timeout
     */
    int _status = 2;

    unsigned int _id;

    void
    Init(SwipeCode &code, DetectorParameters parameters, unsigned int timestamp);


    void SetInstantStart();

protected:
    SwipeCode _code;

    SwypeStepDetector _stepDetector;

    unsigned int _maxTimestamp = 0;

    unsigned int _currentStep = 0;

    bool _relaxed;

    unsigned int _startTimestamp = 0;

    static unsigned int counter;

    double _resultX = 0, _resultY = 0;
};


#endif //SWYPE_SWYPECODEDETECTORBASE_H

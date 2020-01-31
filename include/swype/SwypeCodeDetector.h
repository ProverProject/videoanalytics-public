//
// Created by babay on 20.06.2018.
//

#ifndef SWYPE_SWYPECODEDETECTORBASE_H
#define SWYPE_SWYPECODEDETECTORBASE_H


#include <opencv2/core/mat.hpp>
#include "swype/SwypeStepDetector.h"
#include "swype/SwypeCode.h"

struct SwypeStep {
    unsigned int number;
    char direction;
    unsigned int maxDurationMs;
};

class SwypeCodeDetector {
public:
    SwypeCodeDetector() : _id(++_counter),
                          _status(2),
                          _code(),
                          _stepDetector(_id),
                          _maxTimestamp(0),
                          _currentStep{0, 0, 0},
                          _relaxed(false),
                          _startTimestamp(0),
                          _currentTimestamp(0) {};

    virtual ~SwypeCodeDetector() {};

    void FillResult(int &status, int &index, int &x, int &y, int &message, int &debug) const;

    void FillResult(int &status, int &index, int &message) const;

    /**
     * return current vector size and defect
     * @param point - float[2] -- point coordinates, not null
     * @param defect -- float[2] -- defect size (can be nullptr)
     */
    void GetCurrentVector(float *point, float *defect);

    void Init(SwypeCode &code, DetectorParameters parameters, unsigned int timestamp);

    void Init(DetectorParameters parameters);

    void SetCurrentStep(SwypeStep step, unsigned int firstFrameTimestamp);

    bool IsInitialised() const;

    virtual void Reset(bool resetSwypeCode);

    const SwypeStep &GetCurrentStep() const { return _currentStep; }

    bool IsStepFinished() const { return _status == 1 || _status == 2; };

    virtual void SetSwypeCode(const SwypeCode &code);

    virtual void Start(uint startTimestamp);

protected:

    void AdvanceSwypeStep();

    unsigned int _id;

    /*
     *    3 -- swype code not started (after circle, before swype code)
     *    2 -- swype code completed
     *    1 -- swype step completed
     *    0 -- processing swype code
     *    2 -- waiting to start swype code processing
     *   -1 -- swype code failed
     *   -2 -- swype input timeout
     */
    int _status;

    SwypeCode _code;

    SwypeStepDetector _stepDetector;

    unsigned int _maxTimestamp;

    SwypeStep _currentStep;

    bool _relaxed;

    unsigned int _startTimestamp;

    unsigned int _currentTimestamp;

    static unsigned int _counter;

};


#endif //SWYPE_SWYPECODEDETECTORBASE_H

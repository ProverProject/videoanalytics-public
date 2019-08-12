//
// Created by babay on 29.07.2019.
//

#ifndef PROVER_SWYPECODEDETECTORGAME_H
#define PROVER_SWYPECODEDETECTORGAME_H

#include "SwypeStepDetector.h"
#include "SwypeCodeDetector.h"

class SwypeCodeDetectorDelta2 : public SwypeCodeDetector {
public:

    void SetNextStep(SwypeStep step);

    void NextFrame(VectorExplained shift);

    unsigned int TimeToFail() const;

    void Reset(bool resetSwypeCode) override;

    void AdvanceStep();

    void SetSwypeCode(SwypeCode &code) override;

private:
    SwypeStep _nextStep = {0, 0, 0};
    bool _useSwypeCode = false;
};


#endif //PROVER_SWYPECODEDETECTORGAME_H

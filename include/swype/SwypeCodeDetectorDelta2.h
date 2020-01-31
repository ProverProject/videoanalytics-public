//
// Created by babay on 29.07.2019.
//

#ifndef PROVER_SWYPECODEDETECTORGAME_H
#define PROVER_SWYPECODEDETECTORGAME_H

#include "SwypeStepDetector.h"
#include "SwypeCodeDetector.h"

class SwypeCodeDetectorDelta2 : public SwypeCodeDetector {
public:
    SwypeCodeDetectorDelta2() : _nextStep{0, 0, 0},
                                _useSwypeCode(false) {}

    void SetNextStep(const SwypeStep &step);

    void NextFrame(const VectorExplained &shift);

    unsigned int TimeToFail() const;

    void Reset(bool resetSwypeCode) override;

    void AdvanceStep();

    void SetSwypeCode(const SwypeCode &code) override;

    void Start(uint startTimestamp) override;

private:
    SwypeStep _nextStep;
    bool _useSwypeCode;
};


#endif //PROVER_SWYPECODEDETECTORGAME_H

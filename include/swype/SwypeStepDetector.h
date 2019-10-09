//
// Created by babay on 07.12.2017.
//

#ifndef PROVER_SWYPESTEPDETECTOR_H
#define PROVER_SWYPESTEPDETECTOR_H

#include "swype/VectorExplained.h"
#include "swype/BoundsChecker.h"
#include "swype/DetectorParameters.h"

/**
 * coordinate convention:
 * (0,0) is at current swype-point; 1 is a distance between two swype-points (non diagonal).
 * X axis directed from left to right
 * Y axis directed from top to bottom
 *
 */

class SwypeStepDetector {
public:
    SwypeStepDetector(unsigned int _id) : _id(_id) {}

    virtual void Add(VectorExplained);

    virtual void Set(VectorExplained);

    void Reset();

    /**
     * configures detector's parameters
     */
    void Configure(DetectorParameters parameters);

    /**
     * configures to start movement in specified direction
     * @param dir
     */
    void SetDirection(int dir);

    /**
     * set new direction, taking in account current position and previous targer shift vector
     * @param dir
     */
    void AdvanceDirection(int dir);

    void SetTarget(const VectorExplained &target);

    void FinishStep();

    /**
     * checks current state;
     * @return
     *         1 if we've reached target point
     *         -1 if we've failed to reach point
     *         0 if we're still in progress reaching target point
     */
    int CheckState(bool withDefect);

    VectorExplained _current;

    bool CheckCompleteStep(int direction, const std::vector<VectorExplained> &shifts);

private:
    int _count = 0;
    float _targetRadius = 0;
    DetectorParameters _parameters;
    BoundsChecker _boundsChecker;
    VectorExplained _total;
    VectorExplained _target;
    unsigned int _id;
};


#endif //PROVER_SWYPESTEPDETECTOR_H

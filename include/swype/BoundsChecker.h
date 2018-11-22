//
// Created by babay on 09.12.2017.
//

#ifndef PROVER_MVP_ANDROID_BOUNDSCHECKER_H
#define PROVER_MVP_ANDROID_BOUNDSCHECKER_H


#include "swype/Vector.h"
#include "swype/VectorExplained.h"

// fit-factor for SimplestBoundsCheck, in (0, 1)
// FIT_FACTOR_H is for horizontal and vertical movement;
// FIT_FACTOR_H is for diagonal

#define FIT_FACTOR_H 0.5f
#define FIT_FACTOR_H_RELAXED 0.55f
#define BOUNDS_CHECKER_TARGET_SAFE_ZONE 0.35

/**
 * Checks that we are closer to source or target swype points (and a line between them) then to other swipe points
 *
 * rotates original vector to direction 7 or 8 to simplify calculations
 */

class BoundsChecker {
public:
    void SetDirection(int targetDirection);

    bool CheckBounds(VectorExplained p);

    bool CheckBoundsWithDefect(VectorExplained p);

    void SetTargetRadius(float _targetRadius, float targetRadiusOther);

    BoundsChecker() {}

private:

    void SetTurnMatForDirectionDiff(int directionDiff);

    void SetMatForDirection(int direction);

    bool _isDiagonal;
    double _turnMat[2][2];

    float _targetRadius;
    // target radius for non-target points
    float _targetRadiusOther;
    VectorExplained _target;
};


#endif //PROVER_MVP_ANDROID_BOUNDSCHECKER_H

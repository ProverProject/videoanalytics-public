//
// Created by babay on 20.05.2018.
//

#ifndef PROVER_DETECTORSTATUS_H
#define PROVER_DETECTORSTATUS_H


#include <sys/types.h>
#include "swype/SwypeCode.h"
#include "swype/settings.h"


enum DetectorState {
    /**
     * We don't know the swype-code and waiting for it
     */
            WaitingForCode = 0,

    /**
     * We know the swype-code and waiting for circular move
     */
            WaitingForCircle = 1,

    /**
     * We've got circular move and waiting 1500 ms to start swype-code detection
     */
            WaitingToStartSwypeCode = 2,

    /**
     * swype-code detection in progress
     */
            DetectingSwypeCode = 3,

    /**
     * swype-code detected; all done;
     */
            SwypeCodeDone = 4
};


#endif //PROVER_DETECTORSTATUS_H

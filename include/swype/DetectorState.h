//
// Created by babay on 20.05.2018.
//

#ifndef PROVER_MVP_ANDROID_DETECTORSTATUS_H
#define PROVER_MVP_ANDROID_DETECTORSTATUS_H


#include <sys/types.h>
#include "swype/SwypeCode.h"
#include "swype/settings.h"


class DetectorState {
public:
    enum State {
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

    inline State Status() {
        return _state;
    }

    inline void MoveToState(State state, uint timestamp, SwypeCode &code) {
        _state = state;
        _startTimestamp = timestamp;

        switch (state) {
            case State::WaitingToStartSwypeCode:
                _maxStateEndTime = timestamp + code.PauseBeforeEnterCode();
                break;

            case State::DetectingSwypeCode:
                _maxStateEndTime = timestamp + code.TimeToInput();
                break;

            default:
                _maxStateEndTime = (uint) -1;
        }
    }

    inline void StartDetection(uint timestamp, SwypeCode &code) {
        _state = State::DetectingSwypeCode;
        _startTimestamp = timestamp;
        _maxStateEndTime = timestamp + code.TimeToInput();
    }

    inline bool IsStateOutdated(uint timestamp) {
        return timestamp >= _maxStateEndTime;
    }

private:
    State _state = WaitingForCircle;
    uint _startTimestamp = 0;
    uint _maxStateEndTime = static_cast<uint>(-1);
};



#endif //PROVER_MVP_ANDROID_DETECTORSTATUS_H

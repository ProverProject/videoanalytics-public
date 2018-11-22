//
// Created by babay on 06.01.2018.
//

#ifndef PROVER_MVP_ANDROID_SWIPECODE_H
#define PROVER_MVP_ANDROID_SWIPECODE_H

#include "swype/VectorExplained.h"
#include "swype/settings.h"

class SwipeCode {
public:
    inline SwipeCode() {}

    inline void Init(std::string swype) {
        if (swype == "") {
            _length = 0;
        } else if (swype.at(0) == '5') {
            SetSwypeOld(swype);
        } else if (swype.at(0) == '*') {
            SetSwypeNew(swype);
        }
        if (logLevel > 0) {
            char tmp[17];
            FillDirectionsString(tmp, 17);
            LOGI_NATIVE("Set swype code: %s, directions: %s", swype.c_str(), tmp);
        }
    }

    inline void FillDirectionsString(char buf[], unsigned int maxLen) const {
        --maxLen;
        unsigned int len = _length < maxLen ? _length : maxLen;
        for (unsigned int i = 0; i < len; i++) {
            buf[i] = _directions[i] + '0';
        }
        buf[len] = 0;
    }

    inline bool empty() const { return _length == 0; }

    inline uint PauseBeforeEnterCode() const {
        return PAUSE_TO_ST3;
    }

    inline uint TimeToInput() const {
        return MS_PER_SWIPE_STEP * _length;
    }

    void Append(int direction) {
        if (_length < 16) {
            _directions[_length] = (char) direction;
            ++_length;
        }
    }

    virtual bool Equals(const SwipeCode &other) const {
        if (_length != other._length)
            return false;
        for (unsigned int i = 0; i < _length; ++i) {
            if (_directions[i] != other._directions[i])
                return false;
        }
        return true;
    }

    bool StartsWith(const SwipeCode &other) const {
        if (_length < other._length)
            return false;
        for (unsigned int i = 0; i < other._length; ++i) {
            if (_directions[i] != other._directions[i])
                return false;
        }
        return true;
    }

    void SetCode(const SwipeCode &src) {
        memcpy(_directions, src._directions, 17);
        _length = src._length;
    }

    unsigned int _length = 0;
    char _directions[16];

private:
    void SetSwypeOld(std::string &swype) {
        _length = (unsigned int) (swype.length() - 1);
        if (_length > 16)
            _length = 16;
        int current, prev;
        VectorExplained tmp;
        prev = swype.at(0) - '0';
        for (uint i = 0; i < _length; i++) {
            current = swype.at(i + 1) - '0';
            tmp.SetSwipePoints(prev, current);
            _directions[i] = (char) tmp._direction;
            prev = current;
        }
    }

    void SetSwypeNew(std::string &swype) {
        _length = (unsigned int) (swype.length() - 1);
        if (_length > 16)
            _length = 16;

        for (uint i = 0; i < _length; ++i) {
            _directions[i] = swype.at(i + 1) - '0';
        }
    }
};


#endif //PROVER_MVP_ANDROID_SWIPECODE_H

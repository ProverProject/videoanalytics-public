//
// Created by babay on 06.01.2018.
//

#ifndef PROVER_SWYPECODE_H
#define PROVER_SWYPECODE_H

#include "swype/VectorExplained.h"
#include "swype/settings.h"

class SwypeCode {
public:

    inline SwypeCode() : _length(0) {
    }

    inline SwypeCode(const std::string &swype) : SwypeCode(swype.c_str(), swype.length()) {
    }

    inline SwypeCode(const char *chars, unsigned int length)
            : _length(std::min(
            length == 0 ? 0 : length - 1,
            (uint) MAX_SWYPE_LENGTH)) {
        SetSwypeCode(chars, length);
    }

    SwypeCode(unsigned int length, const int *directions)
            : _length(std::min(length, (uint) MAX_SWYPE_LENGTH)) {
        for (uint i = 0; i < _length; ++i) {
            _directions[i] = (char) directions[i];
        }

        if (logLevel > 0) {
            char tmp[MAX_SWYPE_LENGTH + 1];
            FillDirectionsString(tmp, MAX_SWYPE_LENGTH + 1);
            LOGI_NATIVE("Set swype code directions: %s", tmp);
        }
    }

    inline SwypeCode(const SwypeCode &other) : _length(other._length) {
        memcpy(_directions, other._directions, MAX_SWYPE_LENGTH);
    }

    SwypeCode &operator=(const SwypeCode &other) {
        _length = other._length;
        memcpy(_directions, other._directions, MAX_SWYPE_LENGTH);
        return *this;
    }

    inline void SetSwypeCode(const std::string &swype) {
        SetSwypeCode(swype.c_str(), swype.length());
    }

    inline void SetSwypeCode(const char *chars, unsigned int strLength) {
        if (strLength == 0) {
            _length = 0;
        } else if (chars[0] == '5') {
            SetSwypeV1(chars, strLength);
        } else if (chars[0] == '*') {
            SetSwypeV2(chars, strLength);
        }
        if (logLevel > 0) {
            char tmp[MAX_SWYPE_LENGTH + 1];
            FillDirectionsString(tmp, MAX_SWYPE_LENGTH + 1);
            LOGI_NATIVE("Set swype code: %s, directions: %s", chars, tmp);
        }
    }

    inline void FillDirectionsString(char buf[], unsigned int bufferSize) const {
        --bufferSize;
        unsigned int len = _length < bufferSize ? _length : bufferSize;
        for (unsigned int i = 0; i < len; i++) {
            buf[i] = _directions[i] + '0';
        }
        buf[len] = 0;
    }

    inline bool IsEmpty() const { return _length == 0; }

    inline void Clear() {
        _length = 0;
    };

    inline uint PauseBeforeEnterCode() const {
        return PAUSE_TO_ST3;
    }

    inline uint TimeToInput() const {
        return MS_PER_SWYPE_STEP * _length;
    }

    void Append(int direction) {
        if (_length < MAX_SWYPE_LENGTH) {
            _directions[_length] = (char) direction;
            ++_length;
        }
    }

    virtual bool Equals(const SwypeCode &other) const {
        if (_length != other._length)
            return false;
        for (unsigned int i = 0; i < _length; ++i) {
            if (_directions[i] != other._directions[i])
                return false;
        }
        return true;
    }

    bool StartsWith(const SwypeCode &other) const {
        if (_length < other._length)
            return false;
        for (unsigned int i = 0; i < other._length; ++i) {
            if (_directions[i] != other._directions[i])
                return false;
        }
        return true;
    }

    inline unsigned int Length() const {
        return _length;
    }

    inline char DirectionAt(unsigned int pos) const {
        return _directions[pos];
    }

protected:
    unsigned int _length = 0;
    char _directions[MAX_SWYPE_LENGTH];

private:

    void SetSwypeV1(const char *chars, unsigned int length) {
        _length = length - 1;
        if (_length > MAX_SWYPE_LENGTH)
            _length = MAX_SWYPE_LENGTH;
        int current, prev;
        VectorExplained tmp;
        prev = chars[0] - '0';
        for (uint i = 0; i < _length; i++) {
            current = chars[i + 1] - '0';
            tmp.SetSwypePoints(prev, current);
            _directions[i] = (char) tmp.Direction();
            prev = current;
        }
    }

    void SetSwypeV2(const char *chars, unsigned int length) {
        _length = length - 1;
        if (_length > MAX_SWYPE_LENGTH)
            _length = MAX_SWYPE_LENGTH;

        for (uint i = 0; i < _length; ++i) {
            _directions[i] = chars[i + 1] - '0';
        }
    }
};

#endif //PROVER_SWYPECODE_H

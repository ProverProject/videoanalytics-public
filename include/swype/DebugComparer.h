//
// Created by babay on 30.06.2018.
//

#ifndef SWYPE_DEBUGCOMPARER_H
#define SWYPE_DEBUGCOMPARER_H


#include <cstdint>
#include <cmath>
#include "swype/common.h"

class DebugComparer {
public:
    void SetKeyframesTimestamps(std::vector<int> timestamps){
        _startTimestamp = static_cast<unsigned int>(timestamps.front() - 15);
        _endTimestamp = static_cast<unsigned int>(timestamps.back() + 15);
    };
    inline int32_t Generate(int debugX, int debugY, int timestamp) {
        int32_t res = debugX << 16 | (debugY & 0xFFFF);
        double mul = 1 / 128.0;
        double dx = debugX * mul,
                dy = debugY * mul;

        if (_mulByScale) {
            dx *= _shiftDetector._xMult;
            dy *= _shiftDetector._yMult;
        }
        if (logLevel & LOG_DEBUG_COMPARE_VECTORS) {
            LOGI_NATIVE("#%d, GenDebug: (%.4f, %.4f) res: %x", timestamp, dx, dy, res);
        }
        return res;
    }

    void SetShiftDetector(ShiftDetector shiftDetector) {
        _shiftDetector = shiftDetector;
        _oldShiftDetector.SetDetectorSize(origDetectorWidth, origDetectorHeight,
                                          shiftDetector._videoAspect);
    }

    void PrintCompare(int timestamp, int debugX, int debugY, int32_t oldDebugValue) {
        if (timestamp < _startTimestamp || timestamp > _endTimestamp)
            return;
        if (logLevel & LOG_DEBUG_COMPARE_VECTORS) {
            if (oldDebugValue == -1) {
                LOGI_NATIVE("no value for old debug");
                return;
            }

            int oldX = oldDebugValue >> 16;
            int oldY = oldDebugValue & 0xffff;
            if (oldX & 0x8000)
                oldX |= 0xFFFF0000;
            if (oldY & 0x8000)
                oldY |= 0xFFFF0000;

            double mul = 1 / 128.0;

            double newDebugX = debugX * mul,
                    newDebugY = debugY * mul,
                    oldDebugX = oldX * mul,
                    oldDebugY = oldY * mul;

            if (_mulByScale) {
                newDebugX *= _shiftDetector._xMult;
                newDebugY *= _shiftDetector._yMult;
                oldDebugX *= _oldShiftDetector._xMult;
                oldDebugY *= _oldShiftDetector._yMult;
            }

            double dx_ = (newDebugX - oldDebugX) / oldDebugX;
            double dy_ = (newDebugY - oldDebugY) / oldDebugY;

            _diffSum += dx_ + dy_;
            _sumAmount += 2;
            _absSumOldX += fabs(oldDebugX);
            _absSumOldY += fabs(oldDebugY);
            _absSumNewX += fabs(newDebugX);
            _absSumNewY += fabs(newDebugY);

            double dSumX = fabs(_absSumNewX - _absSumOldX) / _absSumOldX * 100;
            double dSumY = fabs(_absSumNewY - _absSumOldY) / _absSumOldY * 100;
            //LOGI_NATIVE("#%d, debugCompare new %x,%x old %x, %x, debug: %x", timestamp, debugX & 0xff,
            //            debugY & 0xff, oldX, oldY, oldDebugValue);

            LOGI_NATIVE(
                    "#%d, checkDebug (%.3f, %.3f), old (%.3f, %.3f), sumNew : %.2f+%.2f, sumOld: %.2f+%.2f, dSum: %.2f+%.2f",
                    timestamp, newDebugX, newDebugY, oldDebugX, oldDebugY,
                    _absSumNewX, _absSumNewY, _absSumOldX, _absSumOldY, dSumX, dSumY);
        }
    }

private:
    const int origDetectorWidth = 176;
    const int origDetectorHeight = 144;
    ShiftDetector _shiftDetector;
    ShiftDetector _oldShiftDetector;
    double _diffSum = 0;
    int _sumAmount = 0;

    double _absSumOldX = 0;
    double _absSumOldY = 0;
    double _absSumNewX = 0;
    double _absSumNewY = 0;

    const bool _mulByScale = true;
    unsigned int _startTimestamp;
    unsigned int _endTimestamp;
};


#endif //SWYPE_DEBUGCOMPARER_H

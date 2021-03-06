//
// Created by babay on 07.12.2017.
//

#ifndef PROVER_VECTOREXPLAINED_H
#define PROVER_VECTOREXPLAINED_H

#include <opencv2/opencv.hpp>
#include "swype/Vector.h"
#include "swype/common.h"

class VectorExplained : public Vector {
public:
    VectorExplained() : Vector(0.0, 0.0),
                        _direction(0),
                        _defectX(0), _defectY(0), _defectX2sum(0.0),
                        _defectY2sum(0.0) {};

    VectorExplained(double x, double y) : Vector(x, y),
                                          _defectX(0), _defectY(0), _defectX2sum(0.0),
                                          _defectY2sum(0.0) {
        CalculateExplained();
    };

    VectorExplained(double x, double y, unsigned int timestamp) : Vector(x, y, timestamp),
                                                                  _defectX(0), _defectY(0),
                                                                  _defectX2sum(0.0),
                                                                  _defectY2sum(0.0) {
        CalculateExplained();
    };

    VectorExplained(const cv::Point2d &source, double mulX, double mulY, unsigned int timestamp)
            : Vector(source.x * mulX, source.y * mulY, timestamp),
              _defectX(0), _defectY(0), _defectX2sum(0.0), _defectY2sum(0.0) {
        CalculateExplained();
    };

    void Set(const cv::Point2d &other);

    void Set(double x, double y) {
        _x = x;
        _y = y;
        CalculateMod();
        CalculateExplained();
    }

    void ApplyWindow(double windowStart, double windowEnd);

    virtual void Add(const VectorExplained &other);

    inline void Reset();

    /**
     * changes vector length but maintains vector direction
     * @param length
     */
    void SetLength(double length);

    virtual void operator*=(double mul);

    inline void Mul(double xMul, double yMul) {
        if (xMul == yMul) {
            operator*=(xMul);
        } else {
            _x *= xMul;
            _defectX *= xMul;
            _y *= yMul;
            _defectY *= yMul;

            CalculateMod();
            CalculateExplained();
        }
    }

    bool CheckWithinRectWithDefect(float left, float top, float right, float bottom) const;

    inline int DirectionDiff(VectorExplained other) const {
        return (_direction - other._direction + 12) % 8 - 4;
    }

    inline void MulWithDefect(double mat[2][2]) {
        double t = mat[0][0] * _x + mat[0][1] * _y;
        _y = mat[1][0] * _x + mat[1][1] * _y;
        _x = t;
        t = mat[0][0] * _defectX + mat[0][1] * _defectY;
        _defectY = fabsf((float) (mat[1][0] * _defectX + mat[1][1] * _defectY));
        _defectX = fabsf((float) t);
    }

    inline void FlipXY() {
        double t = _x;
        _x = _y;
        _y = t;
        float t2 = _defectX;
        _defectX = _defectY;
        _defectY = t2;
    }

    void Log() const;

    /**
     * calculates angle to another vector, result in [-180, 180]
     * @param other
     * @return
     */
    inline double AngleTo(VectorExplained other) const {
        return fmod(other._angle - _angle + 540.0, 360.0) - 180.0;
    }

    inline VectorExplained operator-(VectorExplained other) {
        VectorExplained result;
        result._x = _x - other._x;
        result._y = _y - other._y;
        result._defectX2sum = other._defectX * other._defectX + _defectX * _defectX;
        result._defectY2sum = other._defectY * other._defectY + _defectY * _defectY;
        result._defectX = sqrtf((float) _defectX2sum);
        result._defectY = sqrtf((float) _defectY2sum);

        return result;
    }

    void setRelativeDefect(double relativeDefect) {
        _defectX = fabsf((float) (_x * relativeDefect));
        _defectY = fabsf((float) (_y * relativeDefect));
    }

    void setDefect(float dx, float dy){
        _defectX = dx;
        _defectY = dy;
    }

    inline Vector ShiftDefectEllipseToTouchLineMagnet() const {
        return ShiftEllipseToTouchLineMagnet(_defectX, _defectY);
    }

    inline Vector ShiftDefectRectToTouchLineMagnet() const {
        return ShiftRectToTouchLineMagnet(_defectX, _defectY);
    }

    inline Vector ShiftDefectEllipseToPointMagnet(float targetX, float targetY, float mul) const {
        return EllipticalShiftMagnet(_defectX * mul, _defectY * mul, targetX, targetY);
    }

    inline Vector ShiftDefectRectToPointMagnet(float targetX, float targetY, float mul) const {
        return RectShiftMagnet(_defectX * mul, _defectY * mul, targetX, targetY);
    }

    /**
     * set 1-sized vector for specified direction
     * @param direction
     */
    void SetDirection(int direction);

    /**
     * set 1-sized vector for specified swype-points movement
     * @param from
     * @param to
     */
    void SetSwypePoints(int from, int to);

    /**
     * return vector length defect
     * @return
     */
    float ModDefect() const {
        if (_mod == 0)
            return 0;

        if (_defectX2sum == 0 && _defectY2sum == 0) {
            float t1 = _defectX * (float) _x;
            float t2 = _defectY * (float) _y;
            return sqrtf(t1 * t1 + t2 * t2);
        } else
            return (float) (sqrt(_x * _x * _defectX2sum + _y * _y * _defectY2sum) / _mod);
    }

    double MinDistanceToWithDefect(const Vector &other) const {
#ifdef RECT_DEFECT
        Vector shifted = ShiftDefectRectToPointMagnet((float) other._x, (float) other._y, 1);
#else
        Vector shifted = EllipticalShiftMagnet(_defectX, _defectY,
                                               static_cast<float>(other.X()),
                                               static_cast<float>(other.Y()));
#endif
/*        if (logLevel & LOG_VECTORS) {
            LOGI_NATIVE(
                    "DistanceWithDefect (%.4f %.4f) +-(%.4f %.4f) shifted (%.4f, %.4f) to (%.4f, %.4f) distance = %.4f",
                    _x, _y, _defectX, _defectY, shifted.X(), shifted.Y(), other.X(), other.Y(),
                    shifted.DistanceTo(other)
            );
        }*/
        return shifted.DistanceTo(other);
/*        double dx = fabs(other._x - _x);
        double dy = fabs(other._y - _y);
        dx = dx < _defectX ? 0.0 : dx - _defectX;
        dy = dy < _defectY ? 0.0 : dy - _defectY;
        return sqrt(dx * dx + dy * dy);*/
    }

    inline double Angle() const {
        return _angle;
    }

    inline int Direction() const {
        return _direction;
    }

    inline float DefectX() const {
        return _defectX;
    }

    inline float DefectY() const {
        return _defectY;
    }

    void toFloatArray(float *output) const {
        output[0] = static_cast<float>(_x);
        output[1] = static_cast<float>(_y);
    }

private:

    double _angle;

    /**
     * 1 -- down, 3 -- left, 5 -- top, 7 -- right, 8 -- bottom-right
     */
    int _direction;

    float _defectX;
    float _defectY;

    double _defectX2sum;
    double _defectY2sum;

    void CalculateExplained();
};

inline void VectorExplained::Reset() {
    _x = 0;
    _y = 0;
    _mod = 0;
    _angle = 0;
    _direction = 0;
    _defectX = 0.0;
    _defectY = 0.0;
    _defectX2sum = 0.0;
    _defectY2sum = 0.0;
}


#endif //PROVER_VECTOREXPLAINED_H

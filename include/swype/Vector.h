//
// Created by babay on 08.12.2017.
//

#ifndef PROVER_VECTOR_H
#define PROVER_VECTOR_H


#include <cmath>

class Vector {
public:

    Vector() : _x(0), _y(0), _timestamp(0), _mod(0) {}

    Vector(double x, double y) : _x(x), _y(y), _timestamp(0), _mod(sqrt(x * x + y * y)) {}

    Vector(double x, double y, unsigned int timestamp) : _x(x), _y(y), _timestamp(timestamp),
                                                         _mod(sqrt(x * x + y * y)) {}

    inline void operator+=(const Vector &other) {
        _x += other._x;
        _y += other._y;
    };

    virtual void operator*=(const double mul) {
        _x *= mul;
        _y *= mul;
    };

    inline Vector operator+(const Vector &other) const {
        return Vector(_x + other._x, _y + other._y);
    }

    inline double operator*(const Vector &other) const {
        return _x * other._x + _y * other._y;
    }

    inline void operator-=(const Vector &other) {
        _x -= other._x;
        _y -= other._y;
    }

    inline double Length() const {
        return sqrt(_x * _x + _y * _y);
    }

    inline double DistanceTo(const Vector &other) const {
        double dx = other._x - _x;
        double dy = other._y - _y;
        return sqrt(dx * dx + dy * dy);
    }

    inline double DistanceTo(float x, float y) const {
        double dx = x - _x;
        double dy = y - _y;
        return sqrt(dx * dx + dy * dy);
    }

    inline void Mul(double mat[2][2]) {
        double x = mat[0][0] * _x + mat[0][1] * _y;
        _y = mat[1][0] * _x + mat[1][1] * _y;
        _x = x;
    }

    inline void CalculateMod() {
        _mod = Length();
    }

    /**
     * Assume we have an ellipse with center at this point and (rx, ry) semi axises
     * find a intersection point of the ellipse and line between (this) and (targetX, targetY)
     * @param dx
     * @param dy
     * @param targetX
     * @param targetY
     * @return intersection point
     */
    Vector EllipticalShift(double rx, double ry, double targetX, double targetY) const;

    /**
     * Assume we have an ellipse with center at this point and (rx, ry) semi axises
     * find a intersection point of the ellipse and line between (this) and (targetX, targetY)
     * @param rx
     * @param ry
     * @param targetX
     * @param targetY
     * @param multiplicator
     * @return target point if it is inside ellipse and intersection point otherwise
     */
    Vector EllipticalShiftMagnet(float rx, float ry, float targetX, float targetY) const;

    Vector RectShiftMagnet(float rx, float ry, float targetX, float targetY) const;

    /**
     * Assume we have an ellipse with center at this point and (a, b) semi axises
     * assume x >= 0 and y <= x
     * assume we have a line y = x
     * find a point of ellipse closemost to the line -- it's a point where line parallel to y=x touches the ellipse
     * return point on a line y=x if y=x intersects ellipse (or (0,0)
     * @return
     */
    Vector ShiftEllipseToTouchLineMagnet(float a, float b) const;

    Vector ShiftRectToTouchLineMagnet(float rx, float ry) const;

    inline unsigned int Timestamp() const {
        return _timestamp;
    }

    inline void SetTimestamp(unsigned int timestamp) {
        _timestamp = timestamp;
    }

    inline double X() const {
        return _x;
    }

    inline double Y() const {
        return _y;
    }


    inline double Mod() const {
        return _mod;
    }


protected:
    double _x;
    double _y;

    /**
     * vector timestamp
     */
    unsigned int _timestamp;

    /**
     * vector length
     */
    double _mod;

    inline void Add(double x, double y) {
        _x += x;
        _y += y;
    }
};


#endif //PROVER_VECTOR_H

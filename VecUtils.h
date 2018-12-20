//
// Created by timashkov on 12/18/18.
//

#ifndef STRATEGY_VECUTILS_H
#define STRATEGY_VECUTILS_H

#include <cmath>

class Vec2 {
private:
    double x, y;
public:
    Vec2() {};

    ~Vec2() {};

    Vec2(double x1, double y1) : x(x1), y(y1) {}

    inline double len() const {
        return sqrt(x * x + y * y);
    }

    inline Vec2 *normalize() {
        double length = len();
        x = x / length;
        y = y / length;
        return this;
    }

    double inline getX() const { return x; }

    double inline getY() const { return y; }

    void setX(double d) {
        x = d;
    }

    void operator+(Vec2 &inc) {
        x += inc.getX();
        y += inc.getY();
    }

    Vec2 sub(Vec2 &dec) {
        return Vec2(x - dec.getX(), y - dec.getY());
    }

    void operator*(double mul) {
        x *= mul;
        y *= mul;
    }

    Vec2 mul(double d) {
        return Vec2(x * d, y * d);
    }

    Vec2 add(Vec2 &dec) {
        return Vec2(x + dec.getX(), y + dec.getY());
    }
};

#endif //STRATEGY_VECUTILS_H

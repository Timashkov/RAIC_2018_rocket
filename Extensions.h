//
//  Extensions.h
//  raic
//
//  Created by Alex on 20/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef Extensions_h
#define Extensions_h

#include <memory>
#include <cmath>
#include "model/Ball.h"

using namespace std;
using namespace model;

class Vec3 {
private:
    double X, Y, Z;
public:
    Vec3(const double &x, const double &y, const double &z) : X(x), Y(y), Z(z) {}

    ~Vec3() {}

    inline double getX() const { return X; }

    inline double getY() const { return Y; }

    inline double getZ() const { return Z; }

    inline Vec3 mul(double k) {
        return Vec3(X * k, Y * k, Z * k);
    }

    inline Vec3 add(Vec3 &v) const {
        return Vec3(X + v.getX(), Y + v.getY(), Z + v.getZ());
    }

    inline double len() const {
        return sqrt(X * X + Y * Y + Z * Z);
    }

    inline Vec3 normalized() const {
        double length = len();
        return Vec3(X / length, Y / length, Z / length);
    }

};


class BallExtended {
private:
    Ball ball;
    unique_ptr<Vec3> position;
    unique_ptr<Vec3> velocity;
public:
    BallExtended(const model::Ball &b) {
        ball = b;
        position = unique_ptr<Vec3>(new Vec3(b.x, b.y, b.z));
        velocity = unique_ptr<Vec3>(new Vec3(b.velocity_x, b.velocity_y, b.velocity_z));
    }

    ~BallExtended() {}

    inline Vec3 &getVelocity() const {
        return *velocity;
    }

    inline Vec3 &getPosition() const {
        return *position;
    }

};

#endif /* Extensions_h */

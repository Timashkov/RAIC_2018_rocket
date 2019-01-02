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
#include <numeric>
#include <vector>
#include "utils.h"
#include "model/Ball.h"

using namespace std;
using namespace model;


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

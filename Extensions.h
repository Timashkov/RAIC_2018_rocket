//
//  Extensions.h
//  raic
//
//  Created by Alex on 20/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef Extensions_h
#define Extensions_h
#include "model/Ball.h"

using namespace std;
using namespace model;

class Vec3{
private:
    double X, Y, Z;
public:
    Vec3(double& x, double& y, double& z):X(x),Y(y),Z(z){}
    ~Vec3(){}
};


class BallExtension{
private:
    Ball ball;
    unique_ptr<Vec3> position;
    unique_ptr<Vec3> velocity;
public:
    BallExtension(model::Ball& b){
        ball = b;
        position = unique_ptr<Vec3>(new Vec3(b.x, b.y, b.z));
        velocity = unique_ptr<Vec3>(new Vec3(b.velocity_x, b.velocity_y, b.velocity_z));
    }
    ~BallExtension(){}
    
    inline Vec3& getVelocity() const{
        return *velocity;
    }
    
    inline Vec3& getPosition() const {
        return *position;
    }
    
};

#endif /* Extensions_h */

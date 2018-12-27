//
// Created by timashkov on 12/27/18.
//

#ifndef MYSTRATEGY_ROLEPARAMETERS_H
#define MYSTRATEGY_ROLEPARAMETERS_H

#include "cvl_vec3.h"

class RoleParameters {
public:
    RoleParameters(): anchorPoint(Vec3::None), robotId(-1) {}
    ~RoleParameters(){}

    Vec3 anchorPoint;
    int robotId;
};


#endif //MYSTRATEGY_ROLEPARAMETERS_H

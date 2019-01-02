//
//  cvl_utlis.cpp
//  raic
//
//  Created by Alex on 23/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "utils.h"
#ifdef LOCAL_RUN
#include <fstream>
std::ofstream myfile;
#endif

void writeLog(const std::stringstream& ss){
#ifdef LOCAL_RUN
    if (!myfile.is_open())
        myfile.open ("example.txt");
    myfile << ss.str() <<std::endl;

#endif
}

void closeLog(){
#ifdef LOCAL_RUN
    if (myfile.is_open())
        myfile.close();
#endif
}

Vec3 Vec3::None = Vec3(-1000,-1000,-1000);

Vec3 min(Vec3 a, Vec3 b){
    return a.len() < b.len() ? a: b;
}

Vec3 max(Vec3 a, Vec3 b){
    return a.len()> b.len()? a: b;
}

Vec3 clamp(Vec3 target, Vec3 lb, Vec3 ub){
    return min(max(target, lb), ub);
}

Vec3 clamp(Vec3 target, double max){
    double current = target.len();
    if (current > max){
        return target.normalized() * max;
    }
    return target;
}

double min(double a, double b){
    return a < b ? a: b;
}

double max(double a, double b){
    return a > b? a: b;
}

double clamp(double target, double lb, double ub){
    return min(max(target, lb), ub);
}

double dot(Vec3 a, Vec3 b){
    return a.getX() * b.getX() + a.getY() * b.getY() + a.getZ() * b.getZ();
}

bool isEqual(double a, double b){
    return abs(a-b) < PRECISION;
}
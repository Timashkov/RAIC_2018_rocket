//
//  CVL_Utils.h
//  raic
//
//  Created by Alex on 18/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef CVL_Utils_h
#define CVL_Utils_h
#ifdef LOCAL_RUN

#include <iostream>
#include <sstream>

#endif

#include <cmath>
#include <numeric>
#include <vector>
#include <string>

const double NEVER_COLLISION = 1000000;
const double PRECISION = 0.000001;
const double EPS = 1e-5;

using namespace std;

bool isEqual(double a, double b);

class Vec3 {
private:
    vector<double> vec;
public:
    Vec3(const double x, const double y, const double z) {
        vec = {x, y, z};
    }

    Vec3(const vector<double> &V) {
        vec = V;
    }

    ~Vec3() {}

    inline double getX() const { return vec[0]; }

    inline double getY() const { return vec[1]; }

    inline double getZ() const { return vec[2]; }

    inline void setX(double a) { vec[0] = a; }

    inline void setY(double a) { vec[1] = a; }

    inline void setZ(double a) { vec[2] = a; }

    inline vector<double> getVec() const { return vec; }

    inline Vec3 mul(double k) {

        return Vec3(vec[0] * k, vec[1] * k, vec[2] * k);
    }

    inline Vec3 add(Vec3 &v) const {

        return Vec3(vec[0] + v.getX(), vec[1] + v.getY(), vec[2] + v.getZ());
    }

    inline Vec3 sub(Vec3 &v) const {
        return Vec3(vec[0] - v.getX(), vec[1] - v.getY(), vec[2] - v.getZ());
    }

    inline double len() const {
        return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    }

    inline Vec3 normalized() const {
        double length = len();
        return Vec3(vec[0] / length, vec[1] / length, vec[2] / length);
    }

    friend Vec3 operator+(Vec3 lhs, const Vec3 &rhs) {
        lhs.setX(lhs.getX() + rhs.getX());
        lhs.setY(lhs.getY() + rhs.getY());
        lhs.setZ(lhs.getZ() + rhs.getZ());

        return lhs;
    }

    friend Vec3 operator-(Vec3 lhs, const Vec3 &rhs) {
        lhs.setX(lhs.getX() - rhs.getX());
        lhs.setY(lhs.getY() - rhs.getY());
        lhs.setZ(lhs.getZ() - rhs.getZ());

        return lhs;
    }

    friend Vec3 operator*(Vec3 lhs, const double &rhs) {
        lhs.setX(lhs.getX() * rhs);
        lhs.setY(lhs.getY() * rhs);
        lhs.setZ(lhs.getZ() * rhs);
        return lhs;
    }

    friend Vec3 operator/(Vec3 lhs, const double &rhs) {
        lhs.setX(lhs.getX() / rhs);
        lhs.setY(lhs.getY() / rhs);
        lhs.setZ(lhs.getZ() / rhs);
        return lhs;
    }

    Vec3 &operator=(const Vec3 &other) {
        if (this != &other) {
            this->setX(other.getX());
            this->setY(other.getY());
            this->setZ(other.getZ());
        }
        return *this;
    }

    friend bool operator==(const Vec3 &lhs, const Vec3 &rhs) {
        return isEqual(lhs.getX(), rhs.getX())
               && isEqual(lhs.getY(), rhs.getY())
               && isEqual(lhs.getZ(), rhs.getZ());
    }

    friend bool operator!=(const Vec3 &lhs, const Vec3 &rhs) {
        return !(lhs == rhs);
    }

    static Vec3 None;

    std::string toString() const {
        string s = "(" + to_string(vec[0]) + ";" + to_string(vec[1]) + ";" + to_string(vec[2]) + ") size=" + to_string(len());
        return s;
    }
};

Vec3 min(Vec3 a, Vec3 b);

Vec3 max(Vec3 a, Vec3 b);

Vec3 clamp(Vec3 target, Vec3 lb, Vec3 ub);

Vec3 clamp(Vec3 target, double max);

double min(double a, double b);

double max(double a, double b);

double clamp(double target, double lb, double ub);

double dot(Vec3 a, Vec3 b);

void writeLog(const std::stringstream &ss);

void closeLog();

#endif /* CVL_Utils_h */

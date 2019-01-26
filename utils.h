//
//  CVL_Utils.h
//  raic
//
//  Created by Alex on 18/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef CVL_Utils_h
#define CVL_Utils_h

#include <cmath>
#include <numeric>
#include <string>

const double NEVER_COLLISION = 1000000;
const double PRECISION = 0.000001;
const double EPS = 1e-5;

using namespace std;

bool isEqual(double a, double b);
bool isEqualHard(double a, double b);

class Vec3 {
private:
    double X,Y,Z;
public:
    Vec3(const double& x, const double& y, const double& z): X(x), Y(y),Z(z) {}

    ~Vec3() {}

    inline double getX() const { return X; }

    inline double getY() const { return Y; }

    inline double getZ() const { return Z; }

    inline void setX(double a) { X = a; }

    inline void setY(double a) { Y = a; }

    inline void setZ(double a) { Z = a; }

    inline void mulAndApply(double k) {
        X *= k;
        Y *= k;
        Z *= k;
    }

    inline void addAndApply(const Vec3 &v) {
        X += v.getX();
        Y += v.getY();
        Z += v.getZ();
    }

    inline void subAndApply(const Vec3 &v) {
        X -= v.getX();
        Y -= v.getY();
        Z -= v.getZ();
    }

    inline double len() const {
        return sqrt(lenPowered2());
    }
    
    inline double lenPowered2() const {
        return X * X + Y * Y + Z * Z;
    }

    inline Vec3 normalized() const {
        double length = len();
        return Vec3(X / length, Y / length, Z / length);
    }
    
    inline void normAndApply() {
        double length = len();
        X /= length;
        Y /= length;
        Z /= length;
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
        string s = "(" + to_string(X) + ";" + to_string(Y) + ";" + to_string(Z) + ") size=" + to_string(len());
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

#endif /* CVL_Utils_h */

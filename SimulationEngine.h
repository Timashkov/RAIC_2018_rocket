//
//  SimulationEngine.hpp
//  raic
//
//  Created by Alex on 31/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef SimulationEngine_h
#define SimulationEngine_h

#include "SimulationEntity.h"
#include "model/Rules.h"
#include <iostream>
#include <memory>

class Dan {
public:
    Dan(double d, const Vec3 &n) : distance(d), normal(n) {}

    ~Dan() {}

    double distance;
    Vec3 normal;
};

class SimulationEngine {
private:
    model::Rules rules;
public:
    SimulationEngine(const model::Rules &rul) {
        rules = rul;
    }

    ~SimulationEngine() {}

    void move(SimulationEntity &e, double delta_time);

    bool collide_entities(SimulationEntity &a, SimulationEntity &b);

    Vec3 collide_with_arena(SimulationEntity &e);

    void moveRobot(SimulationEntity& robot, double delta_time);

    Dan dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal);

    Dan dan_to_sphere_inner(Vec3 point, Vec3 sphere_center, double sphere_radius);

    Dan dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius);

    Dan min(Dan a, Dan b);

    Dan dan_to_arena_quarter(Vec3 point);

    Dan dan_to_arena(Vec3 point);

    inline double random(double min, double max) {
        double f = (double) std::rand() / RAND_MAX;
        return min + f * (max - min);
    };
};

#endif /* SimulationEngine_h */

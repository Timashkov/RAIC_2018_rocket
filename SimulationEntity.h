//
// Created by timashkov on 12/27/18.
//

#ifndef MYSTRATEGY_SIMULATIONENTITY_H
#define MYSTRATEGY_SIMULATIONENTITY_H

#include "utils.h"
#include "model/Action.h"

class Dan {
public:
    Dan():distance(-1), normal(Vec3::None){}
    Dan(double d, const Vec3 &n) : distance(d), normal(n) {}
    
    ~Dan() {}
    
    double distance;
    Vec3 normal;
};

class SimulationEntity {
public:
    SimulationEntity() : position(0, 0, 0), velocity(0, 0, 0), touch_normal(Vec3::None),
                         radius(0), mass(0), radius_change_speed(0), teammate(false),
                         touch(false), nitro(0), alive(false), respawn_ticks(0), arena_e(0.0), action_set(false) {}

    ~SimulationEntity() {}


    Vec3 position;
    Vec3 velocity;
    Vec3 touch_normal;
    model::Action action;

    double radius;
    double mass;
    double radius_change_speed;
    bool touch;
    double nitro;
    bool alive;
    bool teammate;
    int respawn_ticks;
    int id;
    int player_id;
    double arena_e;
    bool action_set;

    Dan danToArena;

    inline void setPosition(Vec3 pos) {
        position.setX(pos.getX());
        position.setY(pos.getY());
        position.setZ(pos.getZ());
    }

    inline void setPosition(double x, double y, double z) {
        position.setX(x);
        position.setY(y);
        position.setZ(z);
    }

    inline void setVelocity(Vec3 vel) {
        velocity.setX(vel.getX());
        velocity.setY(vel.getY());
        velocity.setZ(vel.getZ());
    }

    inline void setVelocity(double x, double y, double z) {
        velocity.setX(x);
        velocity.setY(y);
        velocity.setZ(z);
    }

    inline void setNormal(Vec3 vel) {
        touch_normal.setX(vel.getX());
        touch_normal.setY(vel.getY());
        touch_normal.setZ(vel.getZ());
    }

    inline void setNormal(double x, double y, double z) {
        touch_normal.setX(x);
        touch_normal.setY(y);
        touch_normal.setZ(z);
    }

    friend bool operator==(const SimulationEntity &lhs, const SimulationEntity &rhs) {
        return lhs.id == rhs.id && lhs.position == rhs.position && lhs.velocity == rhs.velocity;
    }
};


#endif //MYSTRATEGY_SIMULATIONENTITY_H

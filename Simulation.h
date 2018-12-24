//
//  Simulation.h
//  raic
//
//  Created by Alex on 22/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef Simulation_h
#define Simulation_h

#include <iostream>
#include <memory>
#include "Extensions.h"
#include "model/Rules.h"
#include "model/Game.h"
#include "model/Robot.h"
#include "cvl_vec3.h"

class Entity {
public:
    Entity() : position(0, 0, 0), velocity(0, 0, 0), action_target_velocity(0, 0, 0), touch_normal(Vec3::None),
               radius(0), mass(0), radius_change_speed(0), action_jump_speed(0), action_use_nitro(false),
               touch(false), nitro(0), alive(false), respawn_ticks(0) {}

    ~Entity() {}

    Vec3 position;
    Vec3 velocity;
    Vec3 touch_normal;
    Vec3 action_target_velocity;

    double radius;
    double mass;
    double radius_change_speed;
    double action_jump_speed;
    bool action_use_nitro;
    bool touch;
    double nitro;
    bool alive;
    int respawn_ticks;
    int id;
    int player_id;

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
};

class Dan {
public:
    Dan(double d, const Vec3& n) : distance(d), normal(n) {}

    ~Dan() {}

    double distance;
    Vec3 normal;
};

class State{
public:
    State(){};
    ~State(){};
    
    Entity ball;
//    vector<Entity> aliens; // Enemy, but 'aliens' looks pijje
//    vector<Entity> robots;
};

class TreeNode{
public:
    
    State state;
    shared_ptr<TreeNode> parent;
    int current_tick;
    TreeNode(const State& st, const int tn,TreeNode* pr = NULL):
    state(st), parent(pr), current_tick(tn)
    {}
    ~TreeNode(){}
    
    vector< unique_ptr<TreeNode> > children;
};

class Simulation {
private:
    bool inited;
    int current_tick;
    unique_ptr<TreeNode> baseNode;
public:
    Simulation() : inited(false), current_tick(0) {}

    ~Simulation() {}

    Rules rules;
    Arena arena;

    vector<Entity> robots;
    vector<Entity> aliens;
    vector<Entity> nitro_packs;

    Dan dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal);

    Dan dan_to_sphere_inner(Vec3 point, Vec3 sphere_center, double sphere_radius);

    Dan dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius);

    Dan min(Dan a, Dan b);

    Dan dan_to_arena_quarter(Vec3 point);

    Dan dan_to_arena(Vec3 &point);

    void collide_entities(Entity &a, Entity &b);

    Vec3 collide_with_arena(Entity &e);

    void tick();

    void move(Entity &e, double delta_time);

    void update(double delta_time);

    inline void goal_scored() {};

    inline double random(double min, double max) {
        double f = (double) std::rand() / RAND_MAX;
        return min + f * (max - min);
    };

    inline bool isInited() const { return inited; };

    void init(const Game &g, const Rules &rul) ;

    void go();

    void moveRobots(double delta_time);
};

#endif /* Simulation_h */

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
#include "Rules.h"
#include "Game.h"

class Entity{
public:
    Entity():position(0,0,0), velocity(0,0,0), action_target_velocity(0,0,0), touch_normal(Vec3::None){}
    ~Entity(){}
    
    double radius;
    Vec3 position;
    double mass;
    Vec3 velocity;
    double radius_change_speed;
    Vec3 touch_normal;
    Vec3 action_target_velocity;
    double action_jump_speed;
    bool action_use_nitro;
    bool touch;
    double nitro;
    bool alive;
    int respawn_ticks;
    double arena_e;
    
    void setPosition(Vec3 pos){
        position.setX(pos.getX());
        position.setY(pos.getY());
        position.setZ(pos.getZ());
    }
};

class Dan{
public:
    Dan(double d, Vec3 n): distance(d), normal(n){}
    ~Dan(){}
    
    double distance;
    Vec3 normal;
};

class Simulation{
public:
    Simulation(){}
    ~Simulation(){}
    
    Rules rules;
    Arena arena;
    
    Entity ball;
    vector<Entity> robots;
    vector<Entity> nitro_packs;
    
    Dan dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal);
    Dan dan_to_sphere_inner(Vec3 point, Vec3 sphere_center,double sphere_radius);
    Dan dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius);
    Dan min(Dan a, Dan b);
    Dan dan_to_arena_quarter(Vec3 point);
    Dan dan_to_arena(Vec3& point);
    void collide_entities(Entity& a,Entity& b);
    Vec3 collide_with_arena(Entity& e);
    void tick();
    void move(Entity& e, double delta_time);
    void update(double delta_time);
    inline void goal_scored(){};
    inline double random(double min, double max){
        double f = (double)std::rand() / RAND_MAX;
            return min + f * (max - min);
    };
};

#endif /* Simulation_h */

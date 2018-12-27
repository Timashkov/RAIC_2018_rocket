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
#include "model/Action.h"
#include "cvl_vec3.h"
#include "RoleParameters.h"
#include <queue>
#include "SimulationEntity.h"



class Dan {
public:
    Dan(double d, const Vec3 &n) : distance(d), normal(n) {}

    ~Dan() {}

    double distance;
    Vec3 normal;
};

class State {
public:
    State() {};

//    State(const State &source) : ball(source.ball), robots(source.robots), nitro_packs(source.nitro_packs),
//                           current_tick(source.current_tick + 1) {}

    ~State() {};

    SimulationEntity ball;
    vector<SimulationEntity> robots;
    vector<SimulationEntity> nitro_packs;
    int current_tick;
};

class TreeNode {
public:

    State state;
    TreeNode * parent;


    TreeNode(const State &st, TreeNode *pr = NULL) :
            state(st), parent(pr) {}

    ~TreeNode() {}

    vector<shared_ptr<TreeNode> > children;
};

class Simulation {
private:
    bool inited;
    int current_tick;

    RoleParameters goalKeeper;

    shared_ptr<TreeNode> baseNode;
    queue<shared_ptr<TreeNode>> processingNodes;
    
    Rules rules;
    Arena arena;

public:
    Simulation() : inited(false), current_tick(0) {}

    ~Simulation() {}

    void init(const Game &g, const Rules &rul, const RoleParameters& goalKeeper);
    
    void start();

    Dan dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal);

    Dan dan_to_sphere_inner(Vec3 point, Vec3 sphere_center, double sphere_radius);

    Dan dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius);

    Dan min(Dan a, Dan b);

    Dan dan_to_arena_quarter(Vec3 point);

    Dan dan_to_arena(Vec3 point);

    void collide_entities(SimulationEntity &a, SimulationEntity &b);

    Vec3 collide_with_arena(SimulationEntity &e);

    void tick(shared_ptr<TreeNode> node);

    void move(SimulationEntity &e, double delta_time);

    void update(shared_ptr<TreeNode> &node, double delta_time);

    void moveRobots(shared_ptr<TreeNode> &node, double delta_time);

    inline void goal_scored() {};

    inline double random(double min, double max) {
        double f = (double) std::rand() / RAND_MAX;
        return min + f * (max - min);
    };

    inline bool isInited() const { return inited; };

    void setTick(int tck){ current_tick = tck;}

    void dumpNode(shared_ptr<TreeNode> node);
};

#endif /* Simulation_h */

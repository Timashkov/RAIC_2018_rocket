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
#include "SimulationEngine.h"

#define never_collision 1000000

class CollisionParams {
public:
    CollisionParams(int t, Vec3 p, int rob_id):tick(t),point(p),robot_id(rob_id){}
    ~CollisionParams() {}
    
    int tick;
    Vec3 point;
    int robot_id;
};

class State {
public:
    State(): ball_collision(nullptr){};

    State(const State &source) : ball(source.ball), robots(source.robots), nitro_packs(source.nitro_packs),
        current_tick(source.current_tick), ball_collision(source.ball_collision), robots_collision(source.robots_collision){}
    ~State() {};

    SimulationEntity ball;
    vector<SimulationEntity> robots;
    vector<SimulationEntity> nitro_packs;
    int current_tick;
    CollisionParams * ball_collision;
    vector<CollisionParams> robots_collision;
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
    vector<RoleParameters> forwards;
    
    shared_ptr<TreeNode> baseNode;
    queue<shared_ptr<TreeNode>> processingNodes;
    
    Rules rules;
    Arena arena;
    std::unique_ptr<SimulationEngine> engine;

public:
    Simulation() : inited(false), current_tick(0) {}

    ~Simulation() {}

    void init(const Game &g, const Rules &rul, const RoleParameters& goalKeeper, const vector<RoleParameters>& forwards);
    
    void start();

    void tick(shared_ptr<TreeNode> node);

    void update(shared_ptr<TreeNode> &node, double delta_time);

    void moveRobots(shared_ptr<TreeNode> &node, double delta_time);

    inline void goal_scored() {};

    inline bool isInited() const { return inited; };

    void setTick(const Game &g);

    void dumpNode(shared_ptr<TreeNode> node);
    
    void adjustRobotsPositions(const vector<Robot>& rs);
    
    void truncTree(shared_ptr<TreeNode> tn);
    
    void simulateNextSteps(shared_ptr<TreeNode> base);
    
    inline TreeNode* getBaseNode() const { return baseNode.get();};
};

#endif /* Simulation_h */

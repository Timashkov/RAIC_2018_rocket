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
#include "model/Rules.h"
#include "model/Game.h"
#include "model/Robot.h"
#include "model/Action.h"
#include "utils.h"
#include "RoleParameters.h"
#include <queue>
#include "SimulationEngine.h"

using namespace model;

class CollisionParams {
public:
    CollisionParams(int t, SimulationEntity &anyEnt, SimulationEntity &rob) :
            tick(t), bounty(0) {
        anyEntity = SimulationEntity(anyEnt);
        robot = SimulationEntity(rob);
    }

    ~CollisionParams() {}

    int tick;
    int bounty;
    SimulationEntity anyEntity;
    SimulationEntity robot;

    friend bool operator==(const CollisionParams &lhs, const CollisionParams &rhs) {
        return lhs.tick == rhs.tick && lhs.bounty == rhs.bounty && lhs.anyEntity == rhs.anyEntity &&
               lhs.robot == rhs.robot;
    }
};

class State {
public:
    State() : current_tick(0),bounty(0), ballHitPosition(Vec3::None) {};

    State(const State &source) :
            ball(source.ball),
            robots(source.robots),
            nitro_packs(source.nitro_packs),
            current_tick(source.current_tick),
            ball_collision(source.ball_collision),
            robots_collision(source.robots_collision),
            ball_wall_collision(source.ball_wall_collision),
            distances(source.distances),
            ballHitPosition(Vec3::None),
            bounty(0) {}

    ~State() {};

    SimulationEntity ball;
    vector<SimulationEntity> robots;
    vector<SimulationEntity> nitro_packs;
    int current_tick;
    vector<CollisionParams> ball_collision;
    vector<CollisionParams> robots_collision;
    vector<Vec3> ball_wall_collision;
    int bounty;
    Vec3 ballHitPosition;
    vector<double> distances;
};

class TreeNode {
public:

    State state;
    TreeNode *parent;

    TreeNode(const State &st, TreeNode *pr = NULL) :
            state(st),
            parent(pr) {}

    TreeNode(const TreeNode * tn):
        state(tn->state), parent(tn->parent){}
    ~TreeNode() {}

    vector<shared_ptr<TreeNode> > children;
};

class Simulation {
private:
    bool inited;
    int current_tick;

    int goalKeeperId;

    int attackerId;
    Vec3 attackerTarget;

    Vec3 defaultGoalKeeperPosition;
    Vec3 goalTarget;

    shared_ptr<TreeNode> baseNode;
    queue<shared_ptr<TreeNode>> processingNodes;

    Rules rules;
    Arena arena;
    std::unique_ptr<SimulationEngine> engine;

public:
    Simulation() :
    inited(false),
    defaultGoalKeeperPosition(Vec3::None),
    goalTarget(Vec3::None),
    attackerTarget(Vec3::None),
    current_tick(0),
    goalKeeperId(-1){
        
    }

    ~Simulation() {}

    void
    init(const Game &g, const Rules &rul, const RoleParameters &goalKeeper, const vector<RoleParameters> &forwards);

    void tickForBall(shared_ptr<TreeNode> parent);

    inline bool isInited() const { return inited; };

    void setTick(const Game &g);

//    void update(shared_ptr<TreeNode> &node, double delta_time);

    void updateForBall(shared_ptr<TreeNode> &node, double delta_time);

    void dumpNode(shared_ptr<TreeNode> node);

    void adjustRobotsPositions(const vector<Robot> &rs);

    void truncTree(shared_ptr<TreeNode> tn);

    void simulateNextSteps(shared_ptr<TreeNode> base);

    inline TreeNode *getBaseNode() const { return baseNode.get(); };

    void calculateNodeBounty(shared_ptr<TreeNode> shared_ptr);

    bool isBallDirectionToGoal(const SimulationEntity &ball, bool myGoal);

    void setInitialState(const Game &g, State &st);

    void setRobotsParameters(const shared_ptr<TreeNode> &node, const Game &g);

    void checkAlternatives(shared_ptr<TreeNode> baseNode);
    
    int checkAchievement(SimulationEntity rr1, Vec3 bptarget, Vec3 excludeTarget,int max_attempts);
    
    Vec3 getHitPosition(Vec3 ball_moment_position);
};

#endif /* Simulation_h */

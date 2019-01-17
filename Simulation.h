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
#include <queue>
#include "SimulationEngine.h"

using namespace model;

struct JumpParams{
    double initialVelocity = -1.0;
    int jump_ticks = -1;
    int run_ticks = -1;
    
    JumpParams(const double vel, const int t):initialVelocity(vel), jump_ticks(t), run_ticks(-1){}
    JumpParams(const double vel, const int t, const int rt):initialVelocity(vel), jump_ticks(t), run_ticks(rt){}
};

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
    State() : current_tick(0), bounty(0), ballHitPosition(Vec3::None) {};

    State(const State &source) :
            ball(source.ball),
            robots(source.robots),
            nitro_packs(source.nitro_packs),
            current_tick(source.current_tick),
//            ball_collision(source.ball_collision),
//            robots_collision(source.robots_collision),
//            ball_wall_collision(source.ball_wall_collision),
//            distances(source.distances),
            ballHitPosition(Vec3::None),
            bounty(0) {}

    ~State() {};

    SimulationEntity ball;
    vector<SimulationEntity> robots;
    vector<SimulationEntity> nitro_packs;
    int current_tick;
//    vector<CollisionParams> ball_collision;
//    vector<CollisionParams> robots_collision;
//    vector<Vec3> ball_wall_collision;
    int bounty;
    Vec3 ballHitPosition;
//    vector<double> distances;
};

class TreeNode {
public:

    State state;
    TreeNode *parent;

    TreeNode(const State &st, TreeNode *pr = NULL) :
            state(st),
            parent(pr) {}

    TreeNode(const TreeNode *tn) :
            state(tn->state), parent(tn->parent) {}

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

    shared_ptr<TreeNode> baseNode;
    queue<shared_ptr<TreeNode>> processingNodes;

    Rules rules;
    Arena arena;
    
    double delta_time;
    double GOAL_THRESHOLD;
    
    double ENEMY_GOAL_INACCESS_Z;
    double ENEMY_GOAL_INACCESS_X;
    
    Vec3 defaultGoalKeeperPosition;
    Vec3 ENEMY_GOAL_TARGET;
    
public:
    std::unique_ptr<SimulationEngine> engine;
    Simulation() :
            inited(false),
            defaultGoalKeeperPosition(Vec3::None),
            ENEMY_GOAL_TARGET(Vec3::None),
            attackerTarget(Vec3::None),
            current_tick(0),
            goalKeeperId(-1),
            delta_time(0.0) {

    }

    ~Simulation() {}

    void
    init(const Game &g, const Rules &rul, const int &goal_keeper_id);

    void tickForBall(shared_ptr<TreeNode> parent);

    inline bool isInited() const { return inited; };

    void setTick(const Game &g);

//    void update(shared_ptr<TreeNode> &node, double delta_time);

    void updateForBall(shared_ptr<TreeNode> &node, double delta_time);

    void adjustRobotsPositions(const vector<Robot> &rs);

    void truncTree(shared_ptr<TreeNode> tn);

    void simulateNextSteps(shared_ptr<TreeNode> base);

    inline TreeNode *getBaseNode() const { return baseNode.get(); };

    void calculateNodeBounty(shared_ptr<TreeNode> shared_ptr);

    bool isBallDirectionToGoal(const SimulationEntity &ball, bool myGoal);

    void setInitialState(const Game &g, State &st);

    void setRobotsParameters(const shared_ptr<TreeNode> &node, const Game &g);

    void checkAlternatives(shared_ptr<TreeNode> baseNode);

    JumpParams checkAchievement(SimulationEntity rr1, Vec3 bptarget, Vec3 excludeTarget, int max_attempts);

    Vec3 getHitPosition(SimulationEntity ball);

    void moveRobotAndAdjustNextNode(SimulationEntity &robot, TreeNode *childNode);
    
    JumpParams getJumpParams(const SimulationEntity &se, Vec3 target);
    
    JumpParams getJumpParamsWithMax(const SimulationEntity &se, Vec3 target);
};

#endif /* Simulation_h */

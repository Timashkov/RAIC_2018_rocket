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
    double initialVelocityY;
    int jump_ticks;
    int jump_micros;
    double fullTimeSec;
    JumpParams():initialVelocityY(-1), jump_ticks(-1), jump_micros(-1), fullTimeSec(-1.0){}
    JumpParams(const double velY, const int t, const int micros):initialVelocityY(velY), jump_ticks(t), jump_micros(micros){
        fullTimeSec = ((double)jump_ticks)/60.0 + ((double)jump_micros)/6000.0;
    }
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
    State() : current_tick(0), bounty(0) {};

    State(const State &source) :
            ball(source.ball),
            robots(source.robots),
            nitro_packs(source.nitro_packs),
            current_tick(source.current_tick),
            bounty(0) {}

    ~State() {};

    SimulationEntity ball;
    vector<SimulationEntity> robots;
    vector<SimulationEntity> nitro_packs;
    int current_tick;
    int bounty;
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

    shared_ptr<TreeNode> baseNode;
    queue<shared_ptr<TreeNode>> processingNodes;

    Rules rules;
    Arena arena;
    
    double delta_time;
    double GOAL_THRESHOLD;
    
    double ENEMY_GOAL_INACCESS_Z;
    double ENEMY_GOAL_INACCESS_X;
    double ENEMY_GOAL_SHADOW_CORNER;
    
    Vec3 defaultGoalKeeperPosition;
    Vec3 ENEMY_GOAL_TARGET;
    double dist_k;
    
public:
    std::unique_ptr<SimulationEngine> engine;
    Simulation() :
            inited(false),
            defaultGoalKeeperPosition(Vec3::None),
            ENEMY_GOAL_TARGET(Vec3::None),
            current_tick(0),
            goalKeeperId(-1),
            delta_time(0.0) {

    }

    ~Simulation() {}

    void
    init(const Game &g, const Rules &rul, const int &goal_keeper_id);

    void tickForBall(shared_ptr<TreeNode> parent, bool& result);

    inline bool isInited() const { return inited; };

    void setTick(const Game &g);

//    void update(shared_ptr<TreeNode> &node, double delta_time);

    void updateForBall(shared_ptr<TreeNode> &node, double delta_time);

    void adjustRobotsPositions(const vector<Robot> &rs);

    void truncTree(shared_ptr<TreeNode> tn);

    void simulateNextSteps(shared_ptr<TreeNode> base);

    inline TreeNode *getBaseNode() const { return baseNode.get(); };

    bool isBallDirectionToGoal(const SimulationEntity &ball, bool myGoal);

    void setInitialState(const Game &g, State &st);

    void setRobotsParameters(const shared_ptr<TreeNode> &node, const Game &g);

    void checkAlternatives(shared_ptr<TreeNode> baseNode);

    double checkAchievement(SimulationEntity rr1, Vec3 bptarget, int max_attempts, vector<SimulationEntity>& route);
    
//    bool checkAchievementWithStop(SimulationEntity rr1, Vec3 bptarget, int max_attempts, vector<SimulationEntity>& route);

    Vec3 getHitPosition(SimulationEntity ball, SimulationEntity robot);

    void moveRobotAndAdjustNextNode(SimulationEntity robot, TreeNode *childNode);
    
    JumpParams getJumpParams(Vec3 targetPoint, bool useMaxVelocity) const;
    
};

#endif /* Simulation_h */

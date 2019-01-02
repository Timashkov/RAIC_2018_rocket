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
#include "utils.h"
#include "RoleParameters.h"
#include <queue>
#include "SimulationEngine.h"


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

};

class State {
public:
    State():bounty(0) {};

    State(const State &source) :
            ball(source.ball),
            robots(source.robots),
            nitro_packs(source.nitro_packs),
            current_tick(source.current_tick),
            ball_collision(source.ball_collision),
            robots_collision(source.robots_collision),
            bounty(0){}

    ~State() {};

    SimulationEntity ball;
    vector<SimulationEntity> robots;
    vector<SimulationEntity> nitro_packs;
    int current_tick;
    vector<CollisionParams> ball_collision;
    vector<CollisionParams> robots_collision;
    int bounty;
};

class TreeNode {
public:

    State state;
    TreeNode *parent;

    TreeNode(const State &st, TreeNode *pr = NULL) :
            state(st),
            parent(pr) {}

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
    Simulation() :
            inited(false),
            current_tick(0) {}

    ~Simulation() {}

    void
    init(const Game &g, const Rules &rul, const RoleParameters &goalKeeper, const vector<RoleParameters> &forwards);

    void start();

    void tick(shared_ptr<TreeNode> node);

    inline bool isInited() const { return inited; };

    void setTick(const Game &g);

    void update(shared_ptr<TreeNode> &node, double delta_time);

    inline void goal_scored() {};

    void dumpNode(shared_ptr<TreeNode> node);

    void adjustRobotsPositions(const vector<Robot> &rs);

    void truncTree(shared_ptr<TreeNode> tn);

    void simulateNextSteps(shared_ptr<TreeNode> base);

    inline TreeNode *getBaseNode() const { return baseNode.get(); };

    void calculateNodeBounty(shared_ptr<TreeNode> shared_ptr);

    bool isBallDirectionToGoal(const SimulationEntity& ball, bool myGoal);

    void checkForAlternativeNodes(const shared_ptr<TreeNode>& examineNode, const shared_ptr<TreeNode>& parent);
};

#endif /* Simulation_h */

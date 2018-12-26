#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Extensions.h"
#include "Simulation.h"

class MyStrategy : public Strategy {
private:
    Simulation sim;
    int goal_keeper_id;
public:
    MyStrategy();
    ~MyStrategy(){}
    
    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
    
    void dumpAction(const model::Action& act);
    
    void dumpTick(const model::Game& game);

    void dumpRobot(const model::Robot &r, const char * caption);
    
    void actAsGoalKeeper(const Robot &me, const Rules &rules, const Game &game, Action &action);
    
    void actAsForward(const Robot &me, const Rules &rules, const Game &game, Action &action);
};

#endif

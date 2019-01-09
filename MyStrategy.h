#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Extensions.h"
#include "Simulation.h"
#include "RoleParameters.h"

class MyStrategy : public Strategy {
private:
    Simulation sim;
    RoleParameters goalKeeper;
    vector<RoleParameters> forwards;
public:
    MyStrategy();
    ~MyStrategy(){}
    
    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
};

#endif

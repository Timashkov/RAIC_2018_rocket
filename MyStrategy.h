#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Simulation.h"

class MyStrategy : public Strategy {
private:
    Simulation sim;
    int goalKeeperId;
public:
    MyStrategy();

    ~MyStrategy() {}

    void
    act(const model::Robot &me, const model::Rules &rules, const model::Game &game, model::Action &action) override;
};

#endif

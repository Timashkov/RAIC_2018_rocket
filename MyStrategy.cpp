#include "MyStrategy.h"
#include <cmath>
#include "utils.h"

using namespace model;

MyStrategy::MyStrategy():goalKeeperId(-1) {

}

void MyStrategy::act(const Robot &me, const Rules &rules, const Game &game, Action &action) {
    // Поэтому, если мы не касаемся земли, будет использовать нитро
    // чтобы как можно быстрее попасть обратно на землю

    if (goalKeeperId == -1 && game.robots.size() > 2) {
        int k = 1000;
        for (Robot r: game.robots) {
            if (r.id < k && r.is_teammate)
                k = r.id;
        }
        goalKeeperId = k;
    }

    if (!sim.isInited()) {
        sim.init(game, rules, goalKeeperId);
    }
    sim.setTick(game);

    if (!me.touch) {
        action.target_velocity_x = 0.0;
        action.target_velocity_y = -rules.MAX_ENTITY_SPEED;
        action.target_velocity_z = 0.0;
        action.jump_speed = 0.0;
        action.use_nitro = true;
        return;
    }

    for (SimulationEntity &se : sim.getBaseNode()->state.robots) {
        if (se.id == me.id) {
            action = se.action;
            return;
        }
    }
}


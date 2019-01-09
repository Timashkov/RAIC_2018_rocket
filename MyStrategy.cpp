#include "MyStrategy.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include "utils.h"

using namespace model;
using namespace std;

MyStrategy::MyStrategy() {

}

void MyStrategy::act(const Robot &me, const Rules &rules, const Game &game, Action &action) {
    // Поэтому, если мы не касаемся земли, будет использовать нитро
    // чтобы как можно быстрее попасть обратно на землю

    if (goalKeeper.robotId == -1 && game.robots.size() > 2) {
        int k = 1000;
        for (Robot r: game.robots) {
            if (r.id < k && r.is_teammate)
                k = r.id;
        }
        goalKeeper.robotId = k;
        goalKeeper.anchorPoint = Vec3(0.0, 0.0, -(rules.arena.depth / 3.0));

        for (Robot rr: game.robots) {
            if (rr.id != goalKeeper.robotId && rr.is_teammate) {
                RoleParameters rp;
                rp.robotId = rr.id;
                rp.anchorPoint = Vec3(game.ball.x, 0.0, game.ball.z);
                forwards.push_back(rp);
            }
        }
    }

    if (!sim.isInited()) {
        sim.init(game, rules, goalKeeper, forwards);
    }
    sim.setTick(game);

    if (!me.touch) {
        cout<< " Robot " << me.id << " not on the ground"<< endl;
        action.target_velocity_x = 0.0;
        action.target_velocity_y = -rules.MAX_ENTITY_SPEED;
        action.target_velocity_z = 0.0;
        action.jump_speed = 0.0;
        action.use_nitro = true;
        return;
    }

    for (SimulationEntity &se : sim.getBaseNode()->state.robots) {
        if (se.id == me.id) {
            cout << " Robot " << me.id << "Position " << me.x << ":" << me.y << ":" << me.z << " action "
                 << se.action.target_velocity_x << ":"
                 << se.action.target_velocity_y << ":" << se.action.target_velocity_z << ":" << se.action.jump_speed
                 << endl;
            action = se.action;
            return;
        }
    }
}


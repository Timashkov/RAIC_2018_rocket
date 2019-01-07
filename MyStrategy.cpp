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
//        sim.start();
    }
    sim.setTick(game);

    dumpTick(game);

    dumpRobot(me, "me");

    if (!me.touch) {
        action.target_velocity_x = 0.0;
        action.target_velocity_y = -rules.MAX_ENTITY_SPEED;
        action.target_velocity_z = 0.0;
        action.jump_speed = 0.0;
        action.use_nitro = true;
        return;
    }

    //    unique_ptr<BallExtended> ballExtended(new BallExtended(game.ball));

    for (SimulationEntity &se : sim.getBaseNode()->state.robots) {
        if (se.id == me.id) {
//            cout<< "move robot "<< me.id << "  on tick "<< sim.getBaseNode()->state.current_tick << endl;
            action = se.action;
            return;
        }
    }
    
}

void MyStrategy::actAsGoalKeeper(const Robot &me, const Rules &rules, const Game &game, Action &action) {

}

void MyStrategy::actAsForward(const Robot &me, const Rules &rules, const Game &game, Action &action) {

}


void MyStrategy::dumpRobot(const model::Robot &r, const char *caption) {
    std::stringstream ss;
    ss << " ROBOT: id: *" << r.id << "* caption:" << caption << "* ";
    ss << " player_id: *" << r.player_id << "*";
    ss << " is_teammate: *" << r.is_teammate << "*";
    ss << " coord: (" << r.x << ";" << r.y << ":" << r.z << ")";
    ss << " velocity: (" << r.velocity_x << ";" << r.velocity_y << ";" << r.velocity_z << ")";
    ss << " radius:" << r.radius;
    ss << " nitro:" << r.nitro_amount;
    ss << " touch:" << r.touch;
    ss << " touch_normal: x:" << r.touch_normal_x << "Y:" << r.touch_normal_y << "z:" << r.touch_normal_z << std::endl;
    writeLog(ss);
}

void MyStrategy::dumpAction(const model::Action &act) {
    std::stringstream ss;
    ss << " Action:target:(" << act.target_velocity_x << ";" << act.target_velocity_y << ";" << act.target_velocity_z
       << ")";
    ss << "jump_speed = " << act.jump_speed << " ; use_nitro" << act.use_nitro << ";" << std::endl;
    writeLog(ss);
}


void MyStrategy::dumpTick(const model::Game &game) {

    std::stringstream ss;
    ss << " tick : " << game.current_tick << endl;
    ss << " BALL radius:" << game.ball.radius;
    ss << " coord:(" << game.ball.x << ";" << game.ball.y << ";" << game.ball.z << ")";
    ss << " velocity:(" << game.ball.velocity_x << ";" << game.ball.velocity_y << ";" << game.ball.velocity_z << ")"
       << std::endl;

    for (Robot r: game.robots) {
        ss << " ROBOT: id: *" << r.id << "* " << "* ";
        ss << " player_id: *" << r.player_id << "*";
        ss << " is_teammate: *" << r.is_teammate << "*";
        ss << " coord: (" << r.x << ";" << r.y << ":" << r.z << ")";
        ss << " velocity: (" << r.velocity_x << ";" << r.velocity_y << ";" << r.velocity_z << ")";
        ss << " radius:" << r.radius;
        ss << " nitro:" << r.nitro_amount;
        ss << " touch:" << r.touch;
        ss << " touch_normal: x:" << r.touch_normal_x << "Y:" << r.touch_normal_y << "z:" << r.touch_normal_z
           << std::endl;
    }

    writeLog(ss);
}

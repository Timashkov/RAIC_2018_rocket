#include "MyStrategy.h"
#include "VecUtils.h"
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
        goalKeeper.anchorPoint = Vec3(0.0, 0.0, -(rules.arena.depth * 2.0 / 5.0));

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
        sim.start();
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

    if (me.id == goalKeeper.robotId) {
        actAsGoalKeeper(me, rules, game, action);
        return;
    }

    actAsForward(me, rules, game, action);
}

void MyStrategy::actAsGoalKeeper(const Robot &me, const Rules &rules, const Game &game, Action &action) {
    // Стратегия защитника (или атакующего, не нашедшего хорошего момента для удара):
    // Будем стоять посередине наших ворот
    Vec2 target_pos = Vec2(0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius);
    // Причем, если мяч движется в сторону наших ворот
    if (game.ball.velocity_z < 0) {
        // Найдем время и место, в котором мяч пересечет линию ворот
        double t = (target_pos.getY() - game.ball.z) / game.ball.velocity_z;
        double x = game.ball.x + game.ball.velocity_x * t;
        // Если это место - внутри ворот
        if (fabs(x) < (rules.arena.goal_width / 2.0)) {
            // То пойдем защищать его
            target_pos.setX(x);
        }
    }

    // Если при прыжке произойдет столкновение с мячом, и мы находимся
    // с той же стороны от мяча, что и наши ворота, прыгнем, тем самым
    // ударив по мячу сильнее в сторону противника
    bool is_jump = sqrt(pow(me.x - game.ball.x, 2)
                        + pow(me.y - game.ball.y, 2)
                        + pow(me.z - game.ball.z, 2)) < rules.BALL_RADIUS + rules.ROBOT_MAX_RADIUS;
    is_jump = is_jump && me.z < game.ball.z;


    // Установка нужных полей для желаемого действия
    Vec2 target_velocity = Vec2(target_pos.getX() - me.x, target_pos.getY() - me.z).mul(rules.ROBOT_MAX_GROUND_SPEED);
    //    action = Action();
    action.target_velocity_x = target_velocity.getX();
    action.target_velocity_y = 0.0;
    action.target_velocity_z = target_velocity.getY();
    action.jump_speed = is_jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0;
    action.use_nitro = false;
}

void MyStrategy::actAsForward(const Robot &me, const Rules &rules, const Game &game, Action &action) {

    // Если при прыжке произойдет столкновение с мячом, и мы находимся
    // с той же стороны от мяча, что и наши ворота, прыгнем, тем самым
    // ударив по мячу сильнее в сторону противника

    for (SimulationEntity &se : sim.getBaseNode()->state.robots) {
        if (se.id == me.id){
            action = se.action;
            return;
        }
    }

//    bool is_jump = sqrt(pow(me.x - game.ball.x, 2)
//                        + pow(me.y - game.ball.y, 2)
//                        + pow(me.z - game.ball.z, 2)) < rules.BALL_RADIUS + rules.ROBOT_MAX_RADIUS;
//    is_jump = is_jump && me.z < game.ball.z;
//
//    //------------
//
//    if (sim.getBaseNode()->state.ball_collision != nullptr) {
//        std::cout << "Attacker knows about collision " << std::endl;
//        Vec3 mePos = Vec3(me.x, me.y, me.z);
//        Vec3 delta_pos = sim.getBaseNode()->state.ball_collision->point.sub(mePos);
//        Vec3 target_velocity = Vec3(delta_pos.getX(), delta_pos.getY(), delta_pos.getZ()).normalized().mul(
//                rules.ROBOT_MAX_GROUND_SPEED);
//        action.target_velocity_x = target_velocity.getX();
//        action.target_velocity_y = target_velocity.getY();
//        action.target_velocity_z = target_velocity.getZ();
//        action.jump_speed = is_jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0;
//        action.use_nitro = false;
//    }

    //    // Стратегия нападающего:
    //    // Просимулирем примерное положение мяча в следующие 10 секунд, с точностью 0.1 секунда
    //    std::cout << "Attacker ID " << me.id << std::endl;
    //    for (int i = 1; i < 100; i++) {
    //        std::cout << i << std::endl;
    //        double t = i * 0.1;
    //        Vec2 ss = Vec2(game.ball.velocity_x, game.ball.velocity_z).mul(t);
    //        std::cout << "ball-vec(" << ss.getX() << ";" << ss.getY() << ")" << std::endl;
    //
    //        Vec2 ball_pos = Vec2(game.ball.x, game.ball.z).add(ss);
    //        std::cout << "ball-pos(" << ball_pos.getX() << ";" << ball_pos.getY() << ")" << std::endl;
    //        // Если мяч не вылетит за пределы арены
    //        // (произойдет столкновение со стеной, которое мы не рассматриваем),
    //        // и при этом мяч будет находится ближе к вражеским воротам, чем робот,
    //        if (ball_pos.getY() > me.y
    //            && fabs(ball_pos.getX()) < (rules.arena.width / 2.0)
    //            && fabs(ball_pos.getY()) < (rules.arena.depth / 2.0)) {
    //            std::cout << "closer to enemy" << std::endl;
    //            // Посчитаем, с какой скоростью робот должен бежать,
    //            // Чтобы прийти туда же, где будет мяч, в то же самое время
    //            Vec2 ss = Vec2(me.x, me.z);
    //            Vec2 delta_pos = Vec2(ball_pos.getX(), ball_pos.getY()).sub(ss);
    //            double need_speed = delta_pos.len() / t;
    //            // Если эта скорость лежит в допустимом отрезке
    //            if (0.5 * rules.ROBOT_MAX_GROUND_SPEED < need_speed
    //                && need_speed < rules.ROBOT_MAX_GROUND_SPEED) {
    //                // То это и будет наше текущее действие
    //                Vec2 target_velocity = Vec2(delta_pos.getX(), delta_pos.getY()).normalize()->mul(need_speed);
    //                action.target_velocity_x = target_velocity.getX();
    //                action.target_velocity_y = 0.0;
    //                action.target_velocity_z = target_velocity.getY();
    //                action.jump_speed = is_jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0;
    //                action.use_nitro = false;
    //            }
    //            return;
    //        }
    //    }


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

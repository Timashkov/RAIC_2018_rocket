
//
//  Simulation.cpp
//  raic
//
//  Created by Alex on 23/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "Simulation.h"
#include "utils.h"
#include <memory>

const int DEPTH = 60;

void Simulation::init(const Game &g, const Rules &rul, const RoleParameters &goalKeeper,
                      const vector<RoleParameters> &forwards) {
    rules = rul;
    std::srand(rul.seed);

    this->goalKeeperId = goalKeeper.robotId;
    this->defaultGoalKeeperPosition = goalKeeper.anchorPoint;

    attackerId = -1;
    arena = rul.arena;
    engine = std::unique_ptr<SimulationEngine>(new SimulationEngine(rul));

    State st;
    setInitialState(g, st);
    baseNode = std::make_shared<TreeNode>(st, nullptr);

    inited = true;
    goalTarget = Vec3(0, arena.goal_height / 2.0, arena.depth / 2.0 + rul.BALL_RADIUS);
    delta_time = 1.0 / rules.TICKS_PER_SECOND;
    delta_time = delta_time / rules.MICROTICKS_PER_TICK;
}

void Simulation::setInitialState(const Game &g, State &st) {

    st.ball.setPosition(g.ball.x, g.ball.y, g.ball.z);
    st.ball.radius = g.ball.radius;
    st.ball.mass = rules.BALL_MASS;
    st.ball.arena_e = rules.BALL_ARENA_E;
    st.current_tick = 0;

    for (Robot rob: g.robots) {
        if (!rob.is_teammate)
            continue;
        SimulationEntity erob;
        erob.id = rob.id;
        erob.player_id = rob.player_id;
        erob.setPosition(rob.x, rob.y, rob.z);
        erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
        erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
        erob.touch = rob.touch;
        erob.mass = rules.ROBOT_MASS;
        erob.radius = rules.ROBOT_RADIUS;
        erob.arena_e = rules.ROBOT_ARENA_E;
        erob.teammate = rob.is_teammate;

        erob.action.target_velocity_x = rob.velocity_x;
        erob.action.target_velocity_y = rob.velocity_y;
        erob.action.target_velocity_z = rob.velocity_z;
        erob.action.jump_speed = 0;
        erob.action.use_nitro = false;
        st.robots.push_back(erob);
    }
}

void Simulation::setTick(const Game &g) {
    current_tick = g.current_tick;
    if (baseNode->state.current_tick != current_tick) {
//        cout << "SetTick " << current_tick << endl;
        if (baseNode->children.size() > 0) {
//            cout << "Call known nodes" << endl;
            shared_ptr<TreeNode> &tn = baseNode->children[baseNode->children.size() - 1];

            if (tn->state.current_tick == current_tick &&
                tn->state.ball.position == Vec3(g.ball.x, g.ball.y, g.ball.z)) {
                bool action_required = false;
                for (const SimulationEntity &se : tn->state.robots) {
                    if (se.teammate && !se.action_set) {
                        action_required = true;
//                        cout << " Action required " << action_required << endl;
                    }
                }
                if (!action_required) {
                    adjustRobotsPositions(g.robots);
                    truncTree(tn);
                    simulateNextSteps(baseNode);
                    return;
                }
            }
        }

//        cout << "Simulation wrong: " << current_tick << endl;

        attackerId = -1;
        baseNode->children.clear();
        baseNode->state.ball.setPosition(g.ball.x, g.ball.y, g.ball.z);
        baseNode->state.ball.setVelocity(g.ball.velocity_x, g.ball.velocity_y, g.ball.velocity_z);
//        cout << "Set ball position " << baseNode->state.ball.position.toString() << " and velocity "
//             << baseNode->state.ball.velocity.toString() << endl;
        baseNode->state.current_tick = g.current_tick;

        setRobotsParameters(baseNode, g);
//        cout << "Start sim" << endl;
        tickForBall(baseNode);
        checkAlternatives(baseNode);
//        cout << "After check alternatives " << current_tick << baseNode->state.current_tick << endl;
    }

}

void Simulation::setRobotsParameters(const shared_ptr<TreeNode> &node, const Game &g) {
    for (Robot rob: g.robots) {
        if (!rob.is_teammate)
            continue;
        for (SimulationEntity &erob: node->state.robots) {
            if (erob.id == rob.id) {
                erob.setPosition(rob.x, rob.y, rob.z);
                erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
                erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
                erob.touch = rob.touch;

                erob.action.target_velocity_x = rob.velocity_x;
                erob.action.target_velocity_y = rob.velocity_y;
                erob.action.target_velocity_z = rob.velocity_y;
                erob.action.jump_speed = 0.0;
                erob.action.use_nitro = false;
            }
        }
    }
}

void Simulation::tickForBall(shared_ptr<TreeNode> parent) {

    State st = State(parent->state);
    st.current_tick++;

    if (st.current_tick > DEPTH + current_tick) {
//        cout << "break sim :" << st.current_tick << "-" << current_tick << " st ball pos "
//             << st.ball.position.toString() << endl;
        return;
    }

    shared_ptr<TreeNode> node = make_shared<TreeNode>(st, parent.get());
    parent->children.push_back(node);

    for (int i = 0; i < rules.MICROTICKS_PER_TICK; i++) {
        updateForBall(node, delta_time);
    }

    processingNodes.push(node);
    calculateNodeBounty(node);

    if (!processingNodes.empty()) {
        shared_ptr<TreeNode> tn = processingNodes.front();
        processingNodes.pop();
        tickForBall(tn);
    }
}

//void Simulation::update(shared_ptr<TreeNode> &node, double delta_time) {
//    // FIXME: after investigations
//    //    shuffle(robots);
//    
//    for (SimulationEntity &robot: node->state.robots) {
//        engine->moveRobot(robot, delta_time);
//    }
//    engine->move(node->state.ball, delta_time);
//    
//    for (int i = 0; i < node->state.robots.size(); i++) {
//        for (int j = 0; j < i - 1; j++) {
//            if (engine->collide_entities(node->state.robots[i], node->state.robots[j])) {
//                CollisionParams cpi = CollisionParams(node->state.current_tick, node->state.robots[i],
//                                                      node->state.robots[j]);
//                bool found = false;
//                for (const CollisionParams &cp : node->state.robots_collision) {
//                    if (cp.anyEntity.id == cpi.anyEntity.id && cp.robot.id == cpi.robot.id && cp.tick == cpi.tick)
//                        found = true;
//                }
//                if (!found) node->state.robots_collision.push_back(cpi);
//            }
//        }
//    }
//    
//    for (SimulationEntity &robot : node->state.robots) {
//        if (engine->collide_entities(robot, node->state.ball)) {
//            bool found = false;
//            CollisionParams cpi = CollisionParams(node->state.current_tick, node->state.ball, robot);
//            for (const CollisionParams &cp: node->state.ball_collision) {
//                
//                if (cp.anyEntity == cpi.anyEntity && cp.robot == cpi.robot && cp.tick == node->state.current_tick) {
//                    found = true;
//                    break;
//                }
//            }
//            if (!found) {
//                node->state.ball_collision.emplace_back(cpi);
//            }
//        }
//        Vec3 collision_normal = engine->collide_with_arena(robot);
//        if (collision_normal == Vec3::None) {
//            robot.touch = false;
//        } else {
//            robot.touch = true;
//            robot.touch_normal = collision_normal;
//        }
//    }
//    
//    Vec3 collisionPoint = engine->collide_with_arena(node->state.ball);
//    node->state.ball_wall_collision.push_back(collisionPoint);
//    
//    
//    if (abs(node->state.ball.position.getZ()) > arena.depth / 2.0 + node->state.ball.radius) {
//        node->state.bounty += (node->state.ball.position.getZ() > 0) ? 100 : -100;
//    }
//    
//    for (SimulationEntity &robot : node->state.robots) {
//        if (robot.nitro == rules.MAX_NITRO_AMOUNT)
//            continue;
//        for (SimulationEntity &pack : node->state.nitro_packs) {
//            if (!pack.alive)
//                continue;
//            if ((robot.position - pack.position).len() <= robot.radius + pack.radius) {
//                robot.nitro = rules.MAX_NITRO_AMOUNT;
//                pack.alive = false;
//                pack.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS;
//            }
//        }
//    }
//}

void Simulation::updateForBall(shared_ptr<TreeNode> &node, double delta_time) {
    engine->move(node->state.ball, delta_time);
    Vec3 collisionPoint = engine->collide_with_arena(node->state.ball);
    node->state.ball_wall_collision.push_back(collisionPoint);
    node->state.ballHitPosition = getHitPosition(node->state.ball.position);
}

void Simulation::calculateNodeBounty(shared_ptr<TreeNode> node) {
    if (!node->state.ball_collision.empty()) {
        for (const CollisionParams &cp: node->state.ball_collision) {
            if (isBallDirectionToGoal(cp.anyEntity, true)) {
                node->state.bounty += -10;
            } else if (isBallDirectionToGoal(cp.anyEntity, false)) {
                node->state.bounty += 10;
            } else if (cp.anyEntity.velocity.getZ() < 0) {
                node->state.bounty += -5;
            } else {
                node->state.bounty += 1;
            }
        }
    }

    for (int i = 0; i < node->state.robots.size(); i++) {
        Vec3 res = node->state.robots[i].position - node->state.ball.position;
        if (node->state.distances.size() <= i) {
            node->state.distances.push_back(res.len());
        } else {
            node->state.distances[i] = res.len();
        }
    }

    TreeNode *tmp = node.get();

    while (tmp->parent != NULL) {
        tmp->parent->state.bounty += tmp->state.bounty;
        tmp = tmp->parent;
    }
}

void Simulation::adjustRobotsPositions(const vector<Robot> &rs) {

}

void Simulation::truncTree(shared_ptr<TreeNode> tn) {
    shared_ptr<TreeNode> old;
    baseNode.swap(old);
    tn.swap(baseNode);
    baseNode->parent = NULL;
}

void Simulation::simulateNextSteps(shared_ptr<TreeNode> base) {

    if (base->children.empty()) {
        tickForBall(base);
        return;
    }

    for (const shared_ptr<TreeNode> &tn : base->children) {
        simulateNextSteps(tn);
    }
}

bool Simulation::isBallDirectionToGoal(const SimulationEntity &ball, bool myGoal) {
    if (myGoal && ball.velocity.getZ() < 0) {
        Vec3 goal_left_side_vector = Vec3(
                -rules.arena.goal_width / 2.0 - ball.position.getX(),
                0.0,
                -rules.arena.depth / 2.0 - rules.BALL_RADIUS -
                ball.position.getZ()).normalized();
        Vec3 goal_right_side_vector = Vec3(
                rules.arena.goal_width / 2.0 - ball.position.getX(),
                0.0,
                -rules.arena.depth / 2.0 - rules.BALL_RADIUS -
                ball.position.getZ()).normalized();

        Vec3 horizontal_velocity = Vec3(ball.velocity.getX(), 0.0, ball.velocity.getZ()).normalized();
        double left_angle = atan(goal_left_side_vector.getX() / goal_left_side_vector.getZ());
        double right_angle = atan(goal_right_side_vector.getX() / goal_right_side_vector.getZ());
        double direct_angle = atan(horizontal_velocity.getX() / horizontal_velocity.getZ());

        return left_angle <= direct_angle && right_angle >= direct_angle;

    } else if (!myGoal && ball.velocity.getZ() > 0) {
        Vec3 goal_left_side_vector = Vec3(
                -rules.arena.goal_width / 2.0 - rules.arena.goal_top_radius - ball.position.getX(),
                0.0,
                -rules.arena.depth / 2.0 -
                ball.position.getZ()).normalized();
        Vec3 goal_right_side_vector = Vec3(
                rules.arena.goal_width / 2.0 - rules.arena.goal_top_radius - ball.position.getX(),
                0.0,
                -rules.arena.depth / 2.0 -
                ball.position.getZ()).normalized();

        Vec3 horizontal_velocity = Vec3(ball.velocity.getX(), 0.0, ball.velocity.getZ()).normalized();
        double left_angle = atan(goal_left_side_vector.getX() / goal_left_side_vector.getZ());
        double right_angle = atan(goal_right_side_vector.getX() / goal_right_side_vector.getZ());
        double direct_angle = atan(horizontal_velocity.getX() / horizontal_velocity.getZ());

        return left_angle <= direct_angle && right_angle >= direct_angle;
    }
    return false;
}

void Simulation::checkAlternatives(shared_ptr<TreeNode> baseNode) {


    //
    //    checkachievement
    //
    //    what is collision: who is better
    //

    vector<double> jumppos = vector<double>{1.29, 1.53, 1.75, 1.98, 2.19, 2.33, 2.58, 2.78, 2.96, 3.13, 3.29, 3.44,
                                            3.59, 3.72};

    double delta_time = 1.0 / 6000.0;

    int ticks_till_collision = -1;
    int on_collision_tick = -1;

    TreeNode *tn = baseNode.get();
//    cout << "Check alternatives for node " << baseNode->state.current_tick << endl;
//    bool skipFurtherAnalyze = true;
//    int skipcount = 0;
    while (tn->children.size() > 0) {
//        cout << "Hit position " << tn->state.ballHitPosition.toString() << " for tick " << tn->state.current_tick
//             << endl;

        for (SimulationEntity &se: tn->state.robots) {
            if (se.teammate &&
                se.touch &&
                (((tn->state.ballHitPosition - se.position).len() < rules.arena.depth / 2.0 && se.id == goalKeeperId) ||
                 se.id != goalKeeperId) &&
                se.position.getZ() < tn->state.ball.position.getZ()) {

//                skipFurtherAnalyze = false;
                Vec3 rp = se.position;
                if (tn->state.ball.danToArena.distance > 4) {
                    ticks_till_collision = -1;
                    on_collision_tick = -1;
                } else {
                    ticks_till_collision = checkAchievement(se, tn->state.ballHitPosition, tn->state.ball.position,
                                                            tn->state.current_tick - current_tick);
                    on_collision_tick = tn->state.current_tick;
                }
                if (ticks_till_collision != -1) {
//                    cout << " Check achievement with current tick " << current_tick << " and will be on "
//                         << on_collision_tick << " ticks unitl collision " << ticks_till_collision << endl;
//                    cout << " For Attacker " << se.id << endl;
                    attackerId = se.id;
                    attackerTarget = tn->state.ballHitPosition;
                    break;
                }
            }
        }
        if (attackerId != -1 /*|| skipFurtherAnalyze*/) {
            break;
        }

        tn = tn->children[tn->children.size() - 1].get();
    }

    if (attackerId == goalKeeperId) {
        for (const SimulationEntity &se: baseNode->state.robots) {
            if (se.teammate && se.id != attackerId) {
                goalKeeperId = se.id;
                break;
            }
        }
    }

    tn = baseNode.get();

//    cout << "ACTIONS " << tn->state.current_tick << endl;

    if (ticks_till_collision != -1 && ticks_till_collision != on_collision_tick - tn->state.current_tick) {
//        cout << "Required adjustment" << endl;

        int target_tick =
                (on_collision_tick - tn->state.current_tick - ticks_till_collision) / 2 + tn->state.current_tick;
//        cout << "Adjust to tick " << target_tick << endl;
        while (tn->children.size() > 0 && tn->state.current_tick <= target_tick) {
            TreeNode *childNode = tn->children[tn->children.size() - 1].get();
            for (SimulationEntity &robot: tn->state.robots) {
                if (robot.velocity.len() < 0.5) {

                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    robot.action.target_velocity_x = 0;
                    robot.action.target_velocity_y = 0;
                    robot.action.target_velocity_z = 0;
                    robot.action_set = true;

                    for (int j = 0; j < 100; j++) {
                        engine->moveRobot(robot, delta_time);

                        Vec3 collision_normal = engine->collide_with_arena(robot);
                        if (collision_normal == Vec3::None) {
                            robot.touch = false;
                        } else {
                            robot.touch = true;
                            robot.touch_normal = collision_normal;
                        }
                    }
                    for (SimulationEntity &childSe: childNode->state.robots) {
                        if (childSe.id == robot.id) {
                            childSe.position = robot.position;
                            childSe.velocity = robot.velocity;
                            childSe.touch = robot.touch;
                        }
                    }

                } else {

                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    Vec3 del = defaultGoalKeeperPosition - robot.position;
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
                    for (int j = 0; j < 100; j++) {
                        engine->moveRobot(robot, delta_time);

                        Vec3 collision_normal = engine->collide_with_arena(robot);
                        if (collision_normal == Vec3::None) {
                            robot.touch = false;
                        } else {
                            robot.touch = true;
                            robot.touch_normal = collision_normal;
                        }
                    }
                    for (SimulationEntity &childSe: childNode->state.robots) {
                        if (childSe.id == robot.id) {
                            childSe.position = robot.position;
                            childSe.velocity = robot.velocity;
                            childSe.touch = robot.touch;
                        }
                    }
                }
            }

            tn = childNode;
        }
    } else if (ticks_till_collision != -1) {
        while (tn->children.size() > 0 && tn->state.current_tick <= on_collision_tick) {
            TreeNode *childNode = tn->children[tn->children.size() - 1].get();
            for (SimulationEntity &robot: tn->state.robots) {
                if (robot.id == attackerId) {

//                    cout << " Go 2 position " << attackerTarget.toString() << " parentNode tick: "
//                         << tn->state.current_tick
//                         << endl;

//                    cout << "New target: " << attackerTarget.toString() << endl;
//                    cout << "Current velocity " << robot.velocity.toString() << " len " << robot.velocity.len() << endl;
//                    cout << "Current position " << robot.position.toString() << endl;

                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    Vec3 del = (attackerTarget - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
//                    cout << "Target velocity " << del.toString() << " len " << del.len() << endl;
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
//                    cout << endl;

                    for (int j = 0; j < 100; j++) {
                        engine->moveRobot(robot, delta_time);

                        Vec3 collision_normal = engine->collide_with_arena(robot);
                        if (collision_normal == Vec3::None) {
                            robot.touch = false;
                        } else {
                            robot.touch = true;
                            robot.touch_normal = collision_normal;
                        }
                    }
                    for (SimulationEntity &childSe: childNode->state.robots) {
                        if (childSe.id == robot.id) {
                            childSe.position = robot.position;
                            childSe.velocity = robot.velocity;
                            childSe.touch = robot.touch;
                        }
                    }

                } else if (robot.id == goalKeeperId) {

                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    Vec3 del = defaultGoalKeeperPosition - robot.position;
                    if (del.len() > 1) {
                        del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    }
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
                    for (int j = 0; j < 100; j++) {
                        engine->moveRobot(robot, delta_time);

                        Vec3 collision_normal = engine->collide_with_arena(robot);
                        if (collision_normal == Vec3::None) {
                            robot.touch = false;
                        } else {
                            robot.touch = true;
                            robot.touch_normal = collision_normal;
                        }
                    }
                    for (SimulationEntity &childSe: childNode->state.robots) {
                        if (childSe.id == robot.id) {
                            childSe.position = robot.position;
                            childSe.velocity = robot.velocity;
                            childSe.touch = robot.touch;
                        }
                    }
                }
            }

            tn = childNode;
        }
    } else {
        TreeNode *targetBallNode = tn;
        for (int j = 0; j < 15; j++) {
            targetBallNode = targetBallNode->children[tn->children.size() - 1].get();
        }
        for (int i = 0; i < 15; i++) {
            TreeNode *childNode = tn->children[tn->children.size() - 1].get();
            for (SimulationEntity &robot: tn->state.robots) {
                if (robot.id == goalKeeperId) {

//                    cout << " Default---- Go 2 position " << defaultGoalKeeperPosition.toString()
//                         << " parentNode tick: "
//                         << tn->state.current_tick
//                         << endl;

                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    Vec3 del = defaultGoalKeeperPosition - robot.position;
                    if (del.len() > 1) {
                        del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    }
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
                    if (del.len() > 0.01) {
                        for (int j = 0; j < 100; j++) {
                            engine->moveRobot(robot, delta_time);

                            Vec3 collision_normal = engine->collide_with_arena(robot);
                            if (collision_normal == Vec3::None) {
                                robot.touch = false;
                            } else {
                                robot.touch = true;
                                robot.touch_normal = collision_normal;
                            }
                        }
                        for (SimulationEntity &childSe: childNode->state.robots) {
                            if (childSe.id == robot.id) {
                                childSe.position = robot.position;
                                childSe.velocity = robot.velocity;
                                childSe.touch = robot.touch;
                            }
                        }

                    }
                } else {

                    Vec3 target = targetBallNode->state.ball.position;
                    target.setZ(target.getZ() - 2);


//                    cout << " Default--- Go 2 ball " << target.toString() << " parentNode tick: "
//                         << tn->state.current_tick
//                         << endl;

                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;

                    Vec3 del = tn->state.ball.position - robot.position;
                    if (del.len() > 1) {
                        del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    }
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
                    for (int j = 0; j < 100; j++) {
                        engine->moveRobot(robot, delta_time);

                        Vec3 collision_normal = engine->collide_with_arena(robot);
                        if (collision_normal == Vec3::None) {
                            robot.touch = false;
                        } else {
                            robot.touch = true;
                            robot.touch_normal = collision_normal;
                        }
                    }
                    for (SimulationEntity &childSe: childNode->state.robots) {
                        if (childSe.id == robot.id) {
                            childSe.position = robot.position;
                            childSe.velocity = robot.velocity;
                            childSe.touch = robot.touch;
                        }
                    }
                }

            }
            tn = childNode;
        }

    }

//    cout << " Finalization " << tn->state.current_tick << endl;
    for (SimulationEntity &se :tn->state.robots) {
        se.action_set = false;
    }
    tn = tn->parent;
//    cout << "ACTIONS DONE on tick " << tn->state.current_tick << endl;

    if (tn->parent != NULL && tn->state.current_tick == on_collision_tick) {
//        cout << " Ball position " << tn->state.ball.position.toString() << " on tick " << on_collision_tick << endl;
        // found last node before collision node
        int i = 0;
        for (; i < jumppos.size(); i++) {
            if (jumppos[i] > tn->state.ball.position.getY())
                break;
        }
//        cout << " Steps before " << i << endl;
        for (SimulationEntity &rob: tn->state.robots) {
            if (rob.id == attackerId) {
                rob.action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
            }
        }

        for (; i > 1 && tn->parent != NULL; i--) {
            tn = tn->parent;
        }
//        cout << " Jump on tick " << tn->state.current_tick << endl;
        for (SimulationEntity &rob: tn->state.robots) {
            if (rob.id == attackerId) {
                rob.action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
            }
        }
    }
}

int Simulation::checkAchievement(SimulationEntity rr1, Vec3 bptarget, Vec3 excludeTarget, int max_attempts) {

    Vec3 initial_delta = bptarget - rr1.position;
    if (delta_time * rules.MICROTICKS_PER_TICK * rules.ROBOT_MAX_GROUND_SPEED * max_attempts < initial_delta.len()) {
        return -1;
    }
    Vec3 action_target_velocity = (bptarget - rr1.position);
    action_target_velocity = action_target_velocity.normalized() * rules.ROBOT_MAX_GROUND_SPEED;

    rr1.action.target_velocity_x = action_target_velocity.getX();
    rr1.action.target_velocity_y = action_target_velocity.getY();
    rr1.action.target_velocity_z = action_target_velocity.getZ();

    int step = 0;
    bool collided = false;
    while (!collided && step <= max_attempts) {

        for (int j = 0; j < rules.MICROTICKS_PER_TICK; j++) {
            engine->moveRobot(rr1, delta_time);

//            Vec3 exclude_delta = rr1.position - excludeTarget;
//            if (exclude_delta.len() < 3.0)
//                return -1;
//
//            cout << "robot position " << rr1.position.toString() << endl;
//            cout << "target " << bptarget.toString() << endl;
            if ((rr1.position - bptarget).len() < 0.1) {
                collided = true;
            }

            Vec3 collision_normal = engine->collide_with_arena(rr1);
            if (collision_normal == Vec3::None) {
                rr1.touch = false;
            } else {
                rr1.touch = true;
                rr1.touch_normal = collision_normal;
            }
        }

//        cout << "Velocity after increase " << rr1.velocity.len() << endl;
        step++;
        if (rr1.velocity.len()> rules.ROBOT_MAX_GROUND_SPEED-0.02) {
            double rest_time = ((rr1.position - bptarget).len() /
                                (rules.ROBOT_MAX_GROUND_SPEED * delta_time * rules.MICROTICKS_PER_TICK));
//            cout << "Rest Time " << rest_time << endl;
            step += rest_time;
//            cout << "Steps " << step << endl;
            if (max_attempts >= step)
                return step;
        }
    }

    if (max_attempts >= step)
        return step;
    return -1;
}

Vec3 Simulation::getHitPosition(Vec3 ball_moment_position) {
    //attacker goes to ball position

    Vec3 goalDirection = goalTarget - ball_moment_position;
    goalDirection.setY(0);
    ball_moment_position.setY(1);

    Vec3 hitPosition =
            ball_moment_position - (goalDirection.normalized() * (rules.ROBOT_RADIUS + rules.BALL_RADIUS - 0.01));
    return hitPosition;
}


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
    this->defaultForwardPosition = Vec3(0, 1, 0);
    this->defaultHalfBackPosition = Vec3(0, 1, -rul.arena.depth / 2);
    for (const Robot &r: g.robots) {
        if (r.is_teammate && r.id != goalKeeper.robotId) {
            if (this->forwardId == -1) {
                this->forwardId = r.id;
            } else {
                this->halfBackId = r.id;
            }
        }

    }
//    this->forwards = forwards;

    arena = rul.arena;
    engine = std::unique_ptr<SimulationEngine>(new SimulationEngine(rul));

    State st;
    setInitialState(g, st);
    baseNode = std::make_shared<TreeNode>(st, nullptr);

    dumpNode(baseNode);
    inited = true;
    goalTarget = Vec3(0, arena.goal_height / 2.0, arena.depth / 2.0 + rul.BALL_RADIUS);
}

void Simulation::setInitialState(const Game &g, State &st) {

    st.ball.setPosition(g.ball.x, g.ball.y, g.ball.z);
    st.ball.radius = g.ball.radius;
    st.ball.mass = rules.BALL_MASS;
    st.ball.arena_e = rules.BALL_ARENA_E;
    st.current_tick = 0;

    for (Robot rob: g.robots) {
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
        cout << "SetTick " << current_tick << endl;
        cout << "Try child" << endl;
        if (baseNode->children.size() > 0) {
            shared_ptr<TreeNode> &tn = baseNode->children[baseNode->children.size() - 1];
            if (tn->state.current_tick == current_tick &&
                tn->state.ball.position == Vec3(g.ball.x, g.ball.y, g.ball.z)) {
                adjustRobotsPositions(g.robots);
                truncTree(tn);
                simulateNextSteps(baseNode);
                return;
            }
        }

        std::cout << "Simulation wrong: " << current_tick << std::endl;

        baseNode->children.clear();
        baseNode->state.ball.setPosition(g.ball.x, g.ball.y, g.ball.z);
        baseNode->state.ball.setVelocity(g.ball.velocity_x, g.ball.velocity_y, g.ball.velocity_z);
        baseNode->state.current_tick = g.current_tick;

        setRobotsParameters(baseNode, g);
        cout << "Start sim" << endl;
        tick(baseNode, 1);

//        if (baseNode->state.bounty < 10) {
        cout << "###################" << endl << "### bounty " << baseNode->state.bounty << endl
             << "###################" << endl;
        checkAlternatives(baseNode);
//        }
    }

}

void Simulation::setRobotsParameters(const shared_ptr<TreeNode> &node, const Game &g) {
    for (Robot rob: g.robots) {
        for (SimulationEntity &erob: node->state.robots) {
            if (erob.id == rob.id) {
                erob.setPosition(rob.x, rob.y, rob.z);
                erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
                erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
                erob.touch = rob.touch;

                // Our robots
//                if (rob.is_teammate) {
//                    erob.action = resolveTargetAction(erob, node.get());
//                } else {
                erob.action.target_velocity_x = rob.velocity_x;
                erob.action.target_velocity_y = rob.velocity_y;
                erob.action.target_velocity_z = rob.velocity_y;
                erob.action.jump_speed = 0.0;
                erob.action.use_nitro = false;
//                }
            }
        }
    }
}


void Simulation::tick(shared_ptr<TreeNode> parent, int calcAttempt) {
    double delta_time = 1.0 / rules.TICKS_PER_SECOND;
    double micro_dt = delta_time / rules.MICROTICKS_PER_TICK;

    State st = State(parent->state);
    st.current_tick++;

    for (SimulationEntity &rob: st.robots) {
        if (rob.teammate) {
            rob.action = resolveTargetAction(rob, parent.get(), calcAttempt);
        }
    }

    if (st.current_tick > DEPTH + current_tick || !st.ball_collision.empty() || st.bounty < -50 || st.bounty > 50
        || st.ball.position.getZ() > rules.arena.depth / 2 + rules.BALL_RADIUS
        || st.ball.position.getZ() < -rules.arena.depth / 2 + rules.BALL_RADIUS) {
        return;
    }

    shared_ptr<TreeNode> node = make_shared<TreeNode>(st, parent.get());
    parent->children.push_back(node);

    for (int i = 0; i < rules.MICROTICKS_PER_TICK; i++) {
        update(node, micro_dt);
    }

    for (SimulationEntity pack : node->state.nitro_packs) {
        if (pack.alive) {
            continue;
        }
        pack.respawn_ticks -= 1;
        if (pack.respawn_ticks == 0)
            pack.alive = true;
    }

    processingNodes.push(node);
//    dumpNode(node);

    calculateNodeBounty(node);

    if (!processingNodes.empty()) {
        shared_ptr<TreeNode> tn = processingNodes.front();
        processingNodes.pop();
        tick(tn, calcAttempt);
    }
}

void Simulation::update(shared_ptr<TreeNode> &node, double delta_time) {
    // FIXME: after investigations
    //    shuffle(robots);

    engine->moveRobots(node->state.robots, delta_time);
    engine->move(node->state.ball, delta_time);

    for (int i = 0; i < node->state.robots.size(); i++) {
        for (int j = 0; j < i - 1; j++) {
            if (engine->collide_entities(node->state.robots[i], node->state.robots[j])) {
                CollisionParams cpi = CollisionParams(node->state.current_tick, node->state.robots[i],
                                                      node->state.robots[j]);
                bool found = false;
                for (const CollisionParams &cp : node->state.robots_collision) {
                    if (cp.anyEntity.id == cpi.anyEntity.id && cp.robot.id == cpi.robot.id && cp.tick == cpi.tick)
                        found = true;
                }
                if (!found) node->state.robots_collision.push_back(cpi);
            }
        }
    }

    for (SimulationEntity &robot : node->state.robots) {
        if (engine->collide_entities(robot, node->state.ball)) {
            std::cout << "Update: ball collision detected" << std::endl;
            bool found = false;
            CollisionParams cpi = CollisionParams(node->state.current_tick, node->state.ball, robot);
            for (const CollisionParams &cp: node->state.ball_collision) {

                if (cp.anyEntity == cpi.anyEntity && cp.robot == cpi.robot && cp.tick == node->state.current_tick) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                node->state.ball_collision.emplace_back(cpi);
            }
        }
        Vec3 collision_normal = engine->collide_with_arena(robot);
        if (collision_normal == Vec3::None) {
            robot.touch = false;
        } else {
            robot.touch = true;
            robot.touch_normal = collision_normal;
        }
    }

    Vec3 collisionPoint = engine->collide_with_arena(node->state.ball);
    node->state.ball_wall_collision.push_back(collisionPoint);


    if (abs(node->state.ball.position.getZ()) > arena.depth / 2.0 + node->state.ball.radius) {
        node->state.bounty += (node->state.ball.position.getZ() > 0) ? 100 : -100;
    }

    for (SimulationEntity &robot : node->state.robots) {
        if (robot.nitro == rules.MAX_NITRO_AMOUNT)
            continue;
        for (SimulationEntity &pack : node->state.nitro_packs) {
            if (!pack.alive)
                continue;
            if ((robot.position - pack.position).len() <= robot.radius + pack.radius) {
                robot.nitro = rules.MAX_NITRO_AMOUNT;
                pack.alive = false;
                pack.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS;
            }
        }
    }
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
    //TODO::: calc collision bounty

    while (tmp->parent != NULL) {
        tmp->parent->state.bounty += tmp->state.bounty;
        tmp = tmp->parent;
    }
}

void Simulation::dumpNode(shared_ptr<TreeNode> node) {
#ifdef LOCAL_RUN
    std::stringstream ss;
    ss << " simulaton:: tick : " << node->state.current_tick << endl;
    ss << " BALL radius:" << node->state.ball.radius;
    ss << " coord:" << node->state.ball.position.toString();
    ss << " velocity:" << node->state.ball.velocity.toString() << std::endl;

    for (const SimulationEntity &r: node->state.robots) {
        ss << " ROBOT: id: " << r.id;
        ss << " player_id: " << r.player_id;
        ss << " coord: " << r.position.toString();
        ss << " velocity: " << r.velocity.toString();
        ss << " radius: " << r.radius;
        ss << " nitro: " << r.nitro;
        ss << " touch: " << r.touch;
        ss << " touch_normal: " << r.touch_normal.toString() << std::endl << std::endl;
    }

    writeLog(ss);
    std::stringstream().swap(ss);
#endif
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
        tick(base, 0);
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

Action Simulation::resolveTargetAction(const SimulationEntity &robot, const TreeNode *parentNode, int calcAttempt) {
    SimulationEntity ball = parentNode->state.ball;

    if (robot.id == goalKeeperId && (ball.velocity.getZ() >= 0 || ball.position.getZ() >= -rules.arena.depth / 4)) {
        Action act;
        act.jump_speed = 0.0;
        act.use_nitro = false;
        Vec3 del = defaultGoalKeeperPosition - robot.position;
        act.target_velocity_x = del.getX();
        act.target_velocity_y = del.getY();
        act.target_velocity_z = del.getZ();

        return act;
    }

    if (calcAttempt == 0) {

        //attacker goes to ball position
        Vec3 goalDirection = goalTarget - ball.position;
        Vec3 ball_dan_norm = (engine->dan_to_arena(ball.position)).normal;

        Vec3 checkPoint = ball.position - ((goalDirection.normalized() - ball_dan_norm) * (robot.radius + ball.radius));

//        Vec3 target = robot.position - ((robot.position - checkPoint).normalized() * (robot.radius + ball.radius));
        Vec3 target = ((checkPoint - robot.position).normalized() * (robot.radius + ball.radius)) + checkPoint;
        double distance = (ball.position - robot.position).len();

        Action act;
        act.jump_speed = (distance < 3.5) ? rules.ROBOT_MAX_JUMP_SPEED : 0.0;
        act.use_nitro = false;
        Vec3 del = (target - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
        act.target_velocity_x = del.getX();
        act.target_velocity_y = del.getY();
        act.target_velocity_z = del.getZ();
        return act;

    } else if (calcAttempt == 1) {
        //attacker goes to ball position
        cout << "Goal Target " << goalTarget.toString() << endl;


        Vec3 goalDirection = goalTarget - ball.position;
        goalDirection.setY(0);
        Vec3 ball_dan_norm = (engine->dan_to_arena(ball.position)).normal;

        Vec3 bp = ball.position;
        bp.setY(1);
        cout << "Goal Direction " << goalDirection.toString() << endl;
        cout << "Ball position " << bp.toString() << endl;
        Vec3 checkPoint = bp - (goalDirection.normalized() * (robot.radius + ball.radius));

        cout << " Check point " << checkPoint.toString() << " parentNode tick: " << parentNode->state.current_tick
             << endl;


        Vec3 target = ((checkPoint - robot.position).normalized() * (robot.radius + ball.radius)) + checkPoint;
        double distance = (bp - robot.position).len();

        cout << "New target: " << target.toString() << endl;
        cout << "Current velocity " << robot.velocity.toString() << " len " << robot.velocity.len() << endl;
        cout << "Current position " << robot.position.toString() << endl;
        cout << "Distance: " << distance << endl;

        Action act;
        act.jump_speed = 0.0;
        act.use_nitro = false;
        Vec3 del = (target - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
        cout << "Target velocity " << del.toString() << " len " << del.len() << endl;
        act.target_velocity_x = del.getX();
        act.target_velocity_y = del.getY();
        act.target_velocity_z = del.getZ();
        cout << endl;
        return act;
    } else /*if (calcAttempt == 3)*/{



        //attacker goes to ball position
        cout << "Ball position " << ball.position.toString() << endl;

        Vec3 goalDirection = goalTarget - ball.position;
        Dan ball_dan_norm = engine->dan_to_arena(ball.position);

        Vec3 plainBallPosition = ball.position;
        plainBallPosition.setY(0.0);

        Vec3 checkPoint = ball.position - (goalDirection.normalized() * (2 * ball.radius)) -
                          (ball_dan_norm.normal * (ball_dan_norm.distance - robot.radius));

        cout << "Check point " << checkPoint.toString() << " parentNode tick: " << parentNode->state.current_tick
             << endl;

        Vec3 target = checkPoint;
        //robot.position - ((robot.position - checkPoint).normalized() * (robot.radius + ball.radius));
        double distance = (plainBallPosition - robot.position).len();

        cout << "New target: " << target.toString() << endl;
        cout << "Current velocity " << robot.velocity.len() << endl;
        cout << "Current position " << robot.position.toString() << endl;
        cout << "Distance: " << distance << endl;

        Action act;
        act.jump_speed = (distance < 3.1) ? rules.ROBOT_MAX_JUMP_SPEED : 0.0;
        act.use_nitro = false;
        Vec3 del = (target - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
        cout << "Target velocity: " << del.toString() << " len:" << del.len() << endl;
        act.target_velocity_x = del.getX();
        act.target_velocity_y = del.getY();
        act.target_velocity_z = del.getZ();
        cout << endl;
        return act;
    }

    // TODO::
    return Action();
}

void Simulation::checkAlternatives(shared_ptr<TreeNode> baseNode) {


//
//    checkachievement
//
//    what is collision: who is better
//




    vector<double> jumppos = vector<double>{1.29, 1.53, 1.75, 1.98, 2.19, 2.33, 2.58, 2.78, 2.96, 3.13, 3.29, 3.44,
                                            3.59, 3.72};




//    tick(baseNode, 1);
    int id = -1;
    double lenn = 1000;
    TreeNode *tn = baseNode.get();
    while (tn->children.size() > 0 && lenn > rules.BALL_RADIUS + rules.ROBOT_RADIUS) {
        for (SimulationEntity &se: tn->state.robots) {
            if (se.teammate && se.id != goalKeeperId && id == -1) {
                Vec3 rp = se.position;
                Vec3 bp = tn->state.ball.position;
                rp.setY(1);
                bp.setY(1);
                lenn = (rp - bp).len();
                id = se.id;
            }
        }

        tn = tn->children[tn->children.size() - 1].get();
    }

    if (tn->parent != NULL) {
        tn = tn->parent;

        // found last node before collision node
        cout << "Tick " << tn->state.current_tick << "ball y:" << tn->state.ball.position.getY() << endl;
        int i = 0;
        for (; i < jumppos.size(); i++) {
            if (jumppos[i] > tn->state.ball.position.getY())
                break;
        }

        for (SimulationEntity &rob: tn->state.robots) {
            if (rob.id == id) {
                rob.action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
            }
        }

        for (i; i > 0 && tn->parent != NULL; i--) {
            tn = tn->parent;
        }
        for (SimulationEntity &rob: tn->state.robots) {
            if (rob.id == id) {
                rob.action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
            }
        }
    }
}


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

void Simulation::init(const Game &g, const Rules &rul, const RoleParameters &goalKeeper,
                      const vector<RoleParameters> &forwards) {
    rules = rul;
    std::srand(rul.seed);

    State st;
    st.ball.setPosition(g.ball.x, g.ball.y, g.ball.z);
    st.ball.radius = g.ball.radius;
    st.ball.mass = rul.BALL_MASS;
    st.ball.arena_e = rules.BALL_ARENA_E;
    st.current_tick = 0;

    arena = rul.arena;
    engine = std::unique_ptr<SimulationEngine>(new SimulationEngine(rul));

    this->goalKeeper = goalKeeper;
    this->forwards = forwards;

    for (Robot rob: g.robots) {
        SimulationEntity erob;
        erob.id = rob.id;
        erob.player_id = rob.player_id;
        erob.setPosition(rob.x, rob.y, rob.z);
        erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
        erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
        erob.touch = rob.touch;
        erob.mass = rul.ROBOT_MASS;
        erob.radius = rul.ROBOT_RADIUS;
        erob.arena_e = rul.ROBOT_ARENA_E;

        if (rob.id == goalKeeper.robotId) {
            Vec3 target_pos = goalKeeper.anchorPoint; // goal keeper default start position
            Vec3 target_velocity = Vec3(target_pos.getX() - rob.x, 0.0, target_pos.getZ() - rob.z);
            erob.action.target_velocity_x = target_velocity.getX();
            erob.action.target_velocity_y = target_velocity.getY();
            erob.action.target_velocity_z = target_velocity.getZ();
            //splitpoint
            erob.action.jump_speed = 0.0;
            erob.action.use_nitro = false;
        } else {
            // Our robots
            for (RoleParameters rp: forwards) {
                if (rp.robotId == rob.id) {
                    Vec3 target_pos = rp.anchorPoint;
                    Vec3 target_velocity = Vec3(target_pos.getX() - rob.x, 0.0,
                                                target_pos.getZ() - rob.z);
                    erob.action.target_velocity_x = target_velocity.getX();
                    erob.action.target_velocity_y = target_velocity.getY();
                    erob.action.target_velocity_z = target_velocity.getZ();
                    //splitpoint
                    erob.action.jump_speed = 0.0;
                    erob.action.use_nitro = false;
                }
            }

        }

        st.robots.push_back(erob);
    }

    baseNode = std::make_shared<TreeNode>(st, nullptr);

    dumpNode(baseNode);
    inited = true;
}

void Simulation::start() {
#ifdef LOCAL_RUN
    std::stringstream ss;
    double delta_time = 1.0 / rules.TICKS_PER_SECOND;
    double micro_dt = delta_time / rules.MICROTICKS_PER_TICK;

    ss << " delta time 1/" << rules.TICKS_PER_SECOND << "=" << 1.0 / rules.TICKS_PER_SECOND << endl;
    ss << " update dt = time/" << rules.MICROTICKS_PER_TICK << "= " << micro_dt << endl;
    ss << " Rules gravity = " << rules.GRAVITY << endl;

    writeLog(ss);
    std::stringstream().swap(ss);
#endif

    tick(baseNode);

#ifdef LOCAL_RUN
    ss << " Simulation done " << endl;
    writeLog(ss);
    std::stringstream().swap(ss);
#endif
}

void Simulation::tick(shared_ptr<TreeNode> parent) {
    double delta_time = 1.0 / rules.TICKS_PER_SECOND;
    double micro_dt = delta_time / rules.MICROTICKS_PER_TICK;

    State st = State(parent->state);
    st.current_tick++;

    for (SimulationEntity &rob: st.robots) {
        //        TODO: expand tree

        if (rob.id == goalKeeper.robotId) {
            Vec3 target_pos = goalKeeper.anchorPoint; // goal keeper default start position
            Vec3 target_velocity = Vec3(target_pos.getX() - rob.position.getX(), 0.0,
                                        target_pos.getZ() - rob.position.getZ());
            rob.action.target_velocity_x = target_velocity.getX();
            rob.action.target_velocity_y = target_velocity.getY();
            rob.action.target_velocity_z = target_velocity.getZ();
            rob.action.jump_speed = 0.0;
            rob.action.use_nitro = false;
        } else {
            // Our robots
            for (const RoleParameters &rp: forwards) {
                if (rp.robotId == rob.id) {
                    Vec3 target_pos = rp.anchorPoint;
                    Vec3 target_velocity = Vec3(target_pos.getX() - rob.position.getX(), 0.0,
                                                target_pos.getZ() - rob.position.getZ());
                    rob.action.target_velocity_x = target_velocity.getX();
                    rob.action.target_velocity_y = target_velocity.getY();
                    rob.action.target_velocity_z = target_velocity.getZ();
                    rob.action.jump_speed = 0.0;
                    rob.action.use_nitro = false;
                }
            }
        }
    }

    if (st.current_tick > 60 + current_tick)
        return;

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
    dumpNode(node);

    calculateNodeBounty(node);

    checkForAlternativeNodes(node, parent);

    if (!processingNodes.empty()) {
        shared_ptr<TreeNode> tn = processingNodes.front();
        processingNodes.pop();
        tick(tn);
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

    engine->collide_with_arena(node->state.ball);

    if (abs(node->state.ball.position.getZ()) > arena.depth / 2.0 + node->state.ball.radius) {
        goal_scored();
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
    TreeNode *tmp = node.get();
    //TODO::: calc collision bounty
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
    while (tmp->parent != NULL) {
        tmp->parent->state.bounty += node->state.bounty;
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

void Simulation::setTick(const Game &g) {
    current_tick = g.current_tick;
    if (baseNode->state.current_tick != current_tick) {
        for (const shared_ptr<TreeNode> &tn: baseNode->children) {
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

        for (Robot rob: g.robots) {
            SimulationEntity erob;
            erob.id = rob.id;
            erob.player_id = rob.player_id;
            erob.setPosition(rob.x, rob.y, rob.z);
            erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
            erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
            erob.touch = rob.touch;

            if (rob.id == goalKeeper.robotId) {
                Vec3 target_pos = goalKeeper.anchorPoint; // goal keeper default start position
                Vec3 target_velocity = Vec3(target_pos.getX() - rob.x, 0.0, target_pos.getZ() - rob.z);
                erob.action.target_velocity_x = target_velocity.getX();
                erob.action.target_velocity_y = target_velocity.getY();
                erob.action.target_velocity_z = target_velocity.getZ();
                erob.action.jump_speed = 0.0;
                erob.action.use_nitro = false;
            } else {
                // Our robots
                for (RoleParameters rp: forwards) {
                    if (rp.robotId == rob.id) {
                        Vec3 target_pos = rp.anchorPoint;
                        Vec3 target_velocity = Vec3(target_pos.getX() - rob.x, 0.0,
                                                    target_pos.getZ() - rob.z);
                        erob.action.target_velocity_x = target_velocity.getX();
                        erob.action.target_velocity_y = target_velocity.getY();
                        erob.action.target_velocity_z = target_velocity.getZ();
                        erob.action.jump_speed = 0.0;
                        erob.action.use_nitro = false;
                    }
                }
            }
            ////sdjflsdjhfgklsjklfjskl
            baseNode->state.robots.push_back(erob);
        }
        tick(baseNode);
        /// TODO:if sim failed ???
    }

}

void Simulation::adjustRobotsPositions(const vector<Robot> &rs) {

}

void Simulation::truncTree(shared_ptr<TreeNode> tn) {
    shared_ptr<TreeNode> old;
    baseNode.swap(old);
    tn.swap(baseNode);
}

void Simulation::simulateNextSteps(shared_ptr<TreeNode> base) {

    if (base->children.empty()) {
        tick(base);
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

void Simulation::checkForAlternativeNodes(const shared_ptr<TreeNode>& examineNode, const shared_ptr<TreeNode>& parent) {

}


//
//  Simulation.cpp
//  raic
//
//  Created by Alex on 23/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "Simulation.h"
#include "CVL_Utils.h"
#include <memory>

void Simulation::init(const Game &g, const Rules &rul, const RoleParameters& goalKeeper, const vector<RoleParameters>& forwards) {
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
            Vec3 target_velocity = Vec3(target_pos.getX()-rob.x, 0.0, target_pos.getZ() - rob.z).normalized().mul(rules.ROBOT_MAX_GROUND_SPEED);
            erob.action.target_velocity_x = target_velocity.getX();
            erob.action.target_velocity_y = target_velocity.getY();
            erob.action.target_velocity_z = target_velocity.getZ();
            erob.action.jump_speed = 0.0;
            erob.action.use_nitro = false;
        } else {
            // Our robots
            for (RoleParameters rp: forwards){
                if (rp.robotId == rob.id){
                    Vec3 target_pos = rp.anchorPoint;
                    Vec3 target_velocity = Vec3(target_pos.getX()-rob.x, 0.0, target_pos.getZ() - rob.z).normalized().mul(rules.ROBOT_MAX_GROUND_SPEED);
                    erob.action.target_velocity_x = target_velocity.getX();
                    erob.action.target_velocity_y = target_velocity.getY();
                    erob.action.target_velocity_z = target_velocity.getZ();
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
            Vec3 target_velocity = Vec3(target_pos.getX()-rob.position.getX(), 0.0, target_pos.getZ() - rob.position.getZ()).normalized().mul(rules.ROBOT_MAX_GROUND_SPEED);
            rob.action.target_velocity_x = target_velocity.getX();
            rob.action.target_velocity_y = target_velocity.getY();
            rob.action.target_velocity_z = target_velocity.getZ();
            rob.action.jump_speed = 0.0;
            rob.action.use_nitro = false;
        } else {
            // Our robots
            for (RoleParameters rp: forwards){
                if (rp.robotId == rob.id){
                    Vec3 target_pos = rp.anchorPoint;
                    Vec3 target_velocity = Vec3(target_pos.getX()-rob.position.getX(), 0.0, target_pos.getZ() - rob.position.getZ()).normalized().mul(rules.ROBOT_MAX_GROUND_SPEED);
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
    
    TreeNode* tmp = node.get();
    //TODO::: push up
    if (node->state.ball_collision != nullptr){
        while (tmp->parent != NULL){
            if (tmp->parent->state.ball_collision == nullptr
                || tmp->parent->state.ball_collision->tick > tmp->state.ball_collision->tick){
                std::cout<<"Ball collision detected "<<node->state.ball_collision->tick<<" checked: "<<current_tick<<std::endl;
                tmp->parent->state.ball_collision = node->state.ball_collision;
                tmp = tmp->parent;
            } else {
                break;
            }
        }
    }

//    Vec3 bp = node->state.ball.position;
//    bp.setX(round( bp.getX() * 100000.0 ) / 100000.0);
//    bp.setY(round( bp.getY() * 100000.0 ) / 100000.0);
//    bp.setZ(round( bp.getZ() * 100000.0 ) / 100000.0);
//    node->state.ball.setPosition(bp);
    
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
    
    if (!processingNodes.empty()) {
        shared_ptr<TreeNode> tn = processingNodes.front();
        processingNodes.pop();
        tick(tn);
    }
}

void Simulation::update(shared_ptr<TreeNode> &node, double delta_time) {
    // FIXME: after investigations
    //    shuffle(robots);
    
    moveRobots(node, delta_time);
    engine->move(node->state.ball, delta_time);
    
    for (int i = 0; i < node->state.robots.size(); i++) {
        for (int j = 0; j < i - 1; j++) {
            if (engine->collide_entities(node->state.robots[i], node->state.robots[j])){
                CollisionParams cpi = CollisionParams(node->state.current_tick, node->state.robots[i].position, node->state.robots[i].id);
                CollisionParams cpj = CollisionParams(node->state.current_tick, node->state.robots[j].position, node->state.robots[j].id);
                bool addi = true;
                bool addj = true;
                for (CollisionParams cp : node->state.robots_collision){
                    if (cp.robot_id == cpi.robot_id)
                        addi = false;
                    if (cp.robot_id == cpj.robot_id)
                        addj = false;
                }
                if (addi) node->state.robots_collision.push_back(cpi);
                if (addj) node->state.robots_collision.push_back(cpj);
            }
        }
    }
    
    for (SimulationEntity &robot : node->state.robots) {
        if (engine->collide_entities(robot, node->state.ball)) {
            std::cout<<"Update: ball collision detected"<<std::endl;
            if(node->state.ball_collision == nullptr){
                node->state.ball_collision = new CollisionParams(node->state.current_tick, robot.position, robot.id);
            } else if(node->state.ball_collision->tick > current_tick){
                node->state.ball_collision = new CollisionParams(node->state.current_tick, robot.position, robot.id);
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

void Simulation::moveRobots(shared_ptr<TreeNode> &node, double delta_time) {
    for (SimulationEntity &robot : node->state.robots) {
        Vec3 action_target_velocity = Vec3(robot.action.target_velocity_x, robot.action.target_velocity_y,
                                           robot.action.target_velocity_z);
        if (robot.touch) {
            Vec3 target_velocity = clamp(action_target_velocity, rules.ROBOT_MAX_GROUND_SPEED);
            target_velocity = target_velocity - (robot.touch_normal * dot(robot.touch_normal, target_velocity));
            Vec3 target_velocity_change = target_velocity - robot.velocity;
            if (target_velocity_change.len() > 0.0) {
                double acceleration = rules.ROBOT_ACCELERATION * max(0.0, robot.touch_normal.getY());
                robot.velocity = robot.velocity + clamp(target_velocity_change.normalized() * acceleration * delta_time,
                                                        target_velocity_change.len());
            }
        }
        
        if (robot.action.use_nitro) {
            Vec3 target_velocity_change = clamp(action_target_velocity - robot.velocity,
                                                robot.nitro * rules.NITRO_POINT_VELOCITY_CHANGE);
            if (target_velocity_change.len() > 0.0) {
                Vec3 acceleration = target_velocity_change.normalized() * rules.ROBOT_NITRO_ACCELERATION;
                Vec3 velocity_change = clamp(acceleration * delta_time, target_velocity_change.len());
                robot.velocity = robot.velocity + velocity_change;
                robot.nitro = robot.nitro - velocity_change.len() / rules.NITRO_POINT_VELOCITY_CHANGE;
            }
        }
        engine->move(robot, delta_time);
        robot.radius = rules.ROBOT_MIN_RADIUS +
        (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed /
        rules.ROBOT_MAX_JUMP_SPEED;
        robot.radius_change_speed = robot.action.jump_speed;
    }
}

void Simulation::dumpNode(shared_ptr<TreeNode> node) {
#ifdef LOCAL_RUN
    std::stringstream ss;
    ss << " simulaton:: tick : " << node->state.current_tick << endl;
    ss << " BALL radius:" << node->state.ball.radius;
    ss << " coord:" << node->state.ball.position.toString();
    ss << " velocity:" << node->state.ball.velocity.toString() << std::endl;
    
    for (SimulationEntity r: node->state.robots) {
        ss << " ROBOT: id: " << r.id;
        ss << " player_id: " << r.player_id;
        ss << " coord: " << r.position.toString();
        ss << " velocity: " << r.velocity.toString();
        ss << " radius: " << r.radius;
        ss << " nitro: " << r.nitro;
        ss << " touch: " << r.touch;
        ss << " touch_normal: " << r.touch_normal.toString()<< std::endl << std::endl;
    }
    
    writeLog(ss);
    std::stringstream().swap(ss);
#endif
}

/*y    double    4.6388979218969197   [1]    double    4.6388979218969197*/

void Simulation::setTick(const Game& g){
    current_tick = g.current_tick;
    if (baseNode->state.current_tick != current_tick){
        for (shared_ptr<TreeNode> tn: baseNode->children){
            if (tn->state.current_tick == current_tick && tn->state.ball.position == Vec3(g.ball.x, g.ball.y, g.ball.z)){
                adjustRobotsPositions(g.robots);
                truncTree(tn);
                simulateNextSteps(baseNode);
                return;
            }
        }
        std::cout<< "Simulation wrong: "<< current_tick << std::endl;
        
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
                Vec3 target_velocity = Vec3(target_pos.getX()-rob.x, 0.0, target_pos.getZ() - rob.z).normalized().mul(rules.ROBOT_MAX_GROUND_SPEED);
                erob.action.target_velocity_x = target_velocity.getX();
                erob.action.target_velocity_y = target_velocity.getY();
                erob.action.target_velocity_z = target_velocity.getZ();
                erob.action.jump_speed = 0.0;
                erob.action.use_nitro = false;
            } else {
                // Our robots
                for (RoleParameters rp: forwards){
                    if (rp.robotId == rob.id){
                        Vec3 target_pos = rp.anchorPoint;
                        Vec3 target_velocity = Vec3(target_pos.getX()-rob.x, 0.0, target_pos.getZ() - rob.z).normalized().mul(rules.ROBOT_MAX_GROUND_SPEED);
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

void Simulation::adjustRobotsPositions(const vector<Robot>& rs){
    
}

void Simulation::truncTree(shared_ptr<TreeNode> tn){
    shared_ptr<TreeNode> old;
    baseNode.swap(old);
    tn.swap(baseNode);
}

void Simulation::simulateNextSteps(shared_ptr<TreeNode> base){
    
    if (base->children.empty()){
        tick(base);
        return;
    }
    
    for (shared_ptr<TreeNode> tn : base->children){
        simulateNextSteps(tn);
    }
}


/*{"seed":2751641718,
 "max_tick_count":18000,
 "arena":{"width":60.0,"height":20.0,"depth":80.0,"bottom_radius":3.0,"top_radius":7.0,"corner_radius":13.0,"goal_top_radius":3.0,"goal_width":30.0,"goal_height":10.0,"goal_depth":10.0,"goal_side_radius":1.0},
 "team_size":2,
 "ROBOT_MIN_RADIUS":1.0,
 "ROBOT_MAX_RADIUS":1.05,
 "ROBOT_MAX_JUMP_SPEED":15.0,
 "ROBOT_ACCELERATION":100.0,
 "ROBOT_NITRO_ACCELERATION":30.0,
 "ROBOT_MAX_GROUND_SPEED":30.0,
 "ROBOT_ARENA_E":0.0,
 "ROBOT_RADIUS":1.0,
 "ROBOT_MASS":2.0,
 "TICKS_PER_SECOND":60,
 "MICROTICKS_PER_TICK":100,
 "RESET_TICKS":120,
 "BALL_ARENA_E":0.7,
 "BALL_RADIUS":2.0,
 "BALL_MASS":1.0,
 "MIN_HIT_E":0.4,
 "MAX_HIT_E":0.5,
 "MAX_ENTITY_SPEED":100.0,
 "MAX_NITRO_AMOUNT":100.0,
 "START_NITRO_AMOUNT":50.0,
 "NITRO_POINT_VELOCITY_CHANGE":0.6,
 "NITRO_PACK_X":20.0,
 "NITRO_PACK_Y":1.0,
 "NITRO_PACK_Z":30.0,
 "NITRO_PACK_RADIUS":0.5,
 "NITRO_PACK_AMOUNT":100.0,
 "NITRO_PACK_RESPAWN_TICKS":600,
 "GRAVITY":30.0}*/

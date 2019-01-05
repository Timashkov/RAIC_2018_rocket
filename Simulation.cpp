
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

    this->goalKeeperId = goalKeeper.robotId;
    this->defaultGoalKeeperPosition = goalKeeper.anchorPoint;
//    this->forwards = forwards;

    arena = rul.arena;
    engine = std::unique_ptr<SimulationEngine>(new SimulationEngine(rul));

    State st;
    setInitialState(g, st);
    baseNode = std::make_shared<TreeNode>(st, nullptr);

    dumpNode(baseNode);
    inited = true;
    goalTarget = Vec3(0, arena.goal_height/2.0, arena.depth/2.0);
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

        setRobotsParameters(baseNode, g);
        tick(baseNode);
        
        if (baseNode->state.bounty < 10){
            cout<<" bounty < 10 "<<endl;
            tick(baseNode);
        }
//        int bounty = baseNode->state.bounty;
//        for (int i = 0; i < 5 && bounty < 10 ; i++){
//            tick(baseNode);
//            bounty = baseNode->state.bounty - bounty;
//        }
    }

}

void Simulation::setRobotsParameters(const shared_ptr<TreeNode> &node, const Game &g) {
    for (Robot rob: g.robots) {
        for (SimulationEntity &erob: node->state.robots){
            if (erob.id == rob.id){
                erob.setPosition(rob.x, rob.y, rob.z);
                erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
                erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
                erob.touch = rob.touch;
                
                // Our robots
                if (rob.is_teammate) {
                    erob.action = resolveTargetAction(erob, node.get());
                } else {
                    erob.action.target_velocity_x = rob.velocity_x;
                    erob.action.target_velocity_y = rob.velocity_y;
                    erob.action.target_velocity_z = rob.velocity_y;
                    erob.action.jump_speed = 0.0;
                    erob.action.use_nitro = false;
                }
            }
            
        }
    }
}


void Simulation::tick(shared_ptr<TreeNode> parent) {
    double delta_time = 1.0 / rules.TICKS_PER_SECOND;
    double micro_dt = delta_time / rules.MICROTICKS_PER_TICK;

    State st = State(parent->state);
    st.current_tick++;

    for (SimulationEntity &rob: st.robots) {
        if (rob.teammate) {
            rob.action = resolveTargetAction(rob, parent.get());
        }
    }

    if (st.current_tick > 60 + current_tick || !st.ball_collision.empty() || st.bounty < -50){
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

    Vec3 collisionPoint = engine->collide_with_arena(node->state.ball);


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
    if (!node->state.ball_collision.empty()){
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
    
    for (int i = 0 ;i < node->state.robots.size(); i++){
        Vec3 res = node->state.robots[i].position - node->state.ball.position;
        if (node->state.distances.size() <= i){
            node->state.distances.push_back(res.len());
        }else{
            node->state.distances[i] = res.len();
        }
    }

    TreeNode *tmp = node.get();
    //TODO::: calc collision bounty

    while (tmp->parent != NULL) {
        tmp->parent->state.bounty += tmp->state.bounty;
//        if (!tmp->state.ball_collision.empty()){
//            for (CollisionParams cp: tmp->state.ball_collision) {
//                bool shouldAdd = true;
//                if (!tmp->parent->state.ball_collision.empty()){
//                    for (const CollisionParams &pcp: tmp->parent->state.ball_collision) {
//                        if (pcp == cp)
//                            shouldAdd = false;
//                    }
//                }
//                if (shouldAdd)
//                    tmp->parent->state.ball_collision.push_back(cp);
//            }
//
//        }
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

Action Simulation::resolveTargetAction(const SimulationEntity& robot, const TreeNode* parentNode) {
    if (parentNode->children.empty()){
        SimulationEntity ball = parentNode->state.ball;

        if (robot.id == goalKeeperId && (ball.velocity.getZ() >= 0 || ball.position.getZ() >= 0)){
            
            Action act;
            act.jump_speed = 0.0;
            act.use_nitro = false;
            Vec3 del = defaultGoalKeeperPosition - robot.position;
            act.target_velocity_x = del.getX();
            act.target_velocity_y = del.getY();
            act.target_velocity_z = del.getZ();
            
            return act;
        }
        //attacker goes to ball position
        cout<< "Goal Target "<< goalTarget.toString() <<endl;
        cout<< "Ball position "<< ball.position.toString() << endl;
        
        Vec3 goalDirection = goalTarget - ball.position;
        Vec3 ball_dan_norm = (engine->dan_to_arena(ball.position)).normal;
        
        Vec3 checkPoint = ball.position - ((goalDirection.normalized()-ball_dan_norm) * (robot.radius + ball.radius));
        
        cout<< " Check point "<< checkPoint.toString() <<" parentNode tick: "<<parentNode->state.current_tick<<endl;
        
        
        Vec3 target = robot.position - ((robot.position - checkPoint).normalized()*(robot.radius + ball.radius));
        double distance = (ball.position-robot.position).len();
        
        cout<< "New target: "<<target.toString()<< endl;
        cout<< "Current velocity "<<robot.velocity.len()<<endl;
        cout<< "Current position "<<robot.position.toString()<<endl;
        cout<< "Distance: "<< distance <<endl;
        
        
        Action act;
        act.jump_speed = (distance < 3.5) ? rules.ROBOT_MAX_JUMP_SPEED:0.0;
        act.use_nitro = false;
        Vec3 del = (target - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
//        if ((engine->dan_to_arena(robot.position)).normal == Vec3(0,1,0)){
//            del.setY(0.0);
//        }
        cout<< "Target velocity "<<del.len()<<endl;
        act.target_velocity_x = del.getX();
        act.target_velocity_y = del.getY();
        act.target_velocity_z = del.getZ();
        cout<<endl;
        return act;

    }

//    const TreeNode* p = parentNode;
//    while( p->children.size() > 0 ){
//        TreeNode* child = p->children[0].get();
//
//
//    }
    
    
    // TODO::
    return Action();
}

/**

move robot 1  on tick 0
move robot 2  on tick 0
Simulation wrong: 1
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.092095;0.000000)
Check point (0.000000;9.173972;-2.998882) parentNode tick: 1
New target: (-4.935030;2.290419;-16.576821)
Current velocity 0
Current position (-5.860171;1.000000;-19.122196)
Distance: 20.6381
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.092095;0.000000)
Check point (0.000000;9.173972;-2.998882) parentNode tick: 1
New target: (-4.935030;2.290419;-16.576821)
Current velocity 0
Current position (-5.860171;1.000000;-19.122196)
Distance: 20.6381
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.079595;0.000000)
Check point (0.000000;9.160536;-2.998908) parentNode tick: 2
New target: (-4.930096;2.289550;-16.563249)
Current velocity 1.66667
Current position (-5.855379;1.000000;-19.109012)
Distance: 20.6214
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.058762;0.000000)
Check point (0.000000;9.138141;-2.998950) parentNode tick: 3
New target: (-4.915761;2.289231;-16.523816)
Current velocity 3.33333
Current position (-5.841099;1.000000;-19.069722)
Distance: 20.5758
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.029595;0.000000)
Check point (0.000000;9.106790;-2.999007) parentNode tick: 4
New target: (-4.892027;2.289463;-16.458523)
Current velocity 5
Current position (-5.817329;1.000000;-19.004324)
Distance: 20.5013
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.992095;0.000000)
Check point (0.000000;9.066480;-2.999078) parentNode tick: 5
New target: (-4.858894;2.290255;-16.367375)
Current velocity 6.66667
Current position (-5.784071;1.000000;-18.912819)
Distance: 20.3978
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.946262;0.000000)
Check point (0.000000;9.017212;-2.999161) parentNode tick: 6
New target: (-4.816364;2.291618;-16.250376)
Current velocity 8.33333
Current position (-5.741324;1.000000;-18.795208)
Distance: 20.2654
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.892095;0.000000)
Check point (0.000000;8.958986;-2.999254) parentNode tick: 7
New target: (-4.764442;2.293572;-16.107536)
Current velocity 10
Current position (-5.689088;1.000000;-18.651490)
Distance: 20.1041
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.829595;0.000000)
Check point (0.000000;8.891802;-2.999355) parentNode tick: 8
New target: (-4.703131;2.296140;-15.938868)
Current velocity 11.6667
Current position (-5.627363;1.000000;-18.481665)
Distance: 19.9139
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.758762;0.000000)
Check point (0.000000;8.815659;-2.999460) parentNode tick: 9
New target: (-4.632438;2.299355;-15.744388)
Current velocity 13.3333
Current position (-5.556148;1.000000;-18.285734)
Distance: 19.6948
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.679595;0.000000)
Check point (0.000000;8.730558;-2.999567) parentNode tick: 10
New target: (-4.552369;2.303253;-15.524116)
Current velocity 15
Current position (-5.475445;1.000000;-18.063695)
Distance: 19.4468
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.592095;0.000000)
Check point (0.000000;8.636498;-2.999671) parentNode tick: 11
New target: (-4.462935;2.307882;-15.278076)
Current velocity 16.6667
Current position (-5.385252;1.000000;-17.815550)
Distance: 19.1698
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.496262;0.000000)
Check point (0.000000;8.533479;-2.999769) parentNode tick: 12
New target: (-4.364146;2.313298;-15.006299)
Current velocity 18.3333
Current position (-5.285570;1.000000;-17.541299)
Distance: 18.864
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.392095;0.000000)
Check point (0.000000;8.421501;-2.999856) parentNode tick: 13
New target: (-4.256015;2.319571;-14.708821)
Current velocity 20
Current position (-5.176398;1.000000;-17.240941)
Distance: 18.5293
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.279595;0.000000)
Check point (0.000000;8.300565;-2.999927) parentNode tick: 14
New target: (-4.138559;2.326782;-14.385689)
Current velocity 21.6667
Current position (-5.057736;1.000000;-16.914476)
Distance: 18.1658
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.158762;0.000000)
Check point (0.000000;8.170669;-2.999976) parentNode tick: 15
New target: (-4.011798;2.335029;-14.036956)
Current velocity 23.3333
Current position (-4.929584;1.000000;-16.561905)
Distance: 17.7734
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.029595;0.000000)
Check point (0.000000;8.031815;-2.999999) parentNode tick: 16
New target: (-3.875757;2.344432;-13.662690)
Current velocity 25
Current position (-4.791942;1.000000;-16.183228)
Distance: 17.3522
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.892095;0.000000)
Check point (0.000000;7.884003;-2.999989) parentNode tick: 17
New target: (-3.730467;2.355133;-13.262974)
Current velocity 26.6667
Current position (-4.644809;1.000000;-15.778444)
Distance: 16.9021
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.746262;0.000000)
Check point (0.000000;7.727232;-2.999940) parentNode tick: 18
New target: (-3.580032;2.366282;-12.849098)
Current velocity 26.7649
Current position (-4.492432;1.000000;-15.359237)
Distance: 16.4354
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.592095;0.000000)
Check point (0.000000;7.561504;-2.999844) parentNode tick: 19
New target: (-3.429774;2.376572;-12.435699)
Current velocity 26.7082
Current position (-4.340362;1.000000;-14.940870)
Distance: 15.9678
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.429595;0.000000)
Check point (0.000000;7.386819;-2.999695) parentNode tick: 20
New target: (-3.279675;2.385931;-12.022719)
Current velocity 26.6553
Current position (-4.188594;1.000000;-14.523332)
Distance: 15.4995
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.258762;0.000000)
Check point (0.000000;7.203179;-2.999485) parentNode tick: 21
New target: (-3.129692;2.394277;-11.610040)
Current velocity 26.6068
Current position (-4.037105;1.000000;-14.106556)
Distance: 15.0304
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.079595;0.000000)
Check point (0.000000;7.010583;-2.999206) parentNode tick: 22
New target: (-2.979778;2.401520;-11.197525)
Current velocity 26.5631
Current position (-3.885867;1.000000;-13.690464)
Distance: 14.5607
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.892095;0.000000)
Check point (0.000000;6.809034;-2.998850) parentNode tick: 23
New target: (-2.829881;2.407552;-10.785025)
Current velocity 26.525
Current position (-3.734851;1.000000;-13.274970)
Distance: 14.0904
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.696262;0.000000)
Check point (0.000000;6.598534;-2.998408) parentNode tick: 24
New target: (-2.679942;2.412246;-10.372367)
Current velocity 26.493
Current position (-3.584022;1.000000;-12.859977)
Distance: 13.6196
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.492095;0.000000)
Check point (0.000000;6.379083;-2.997871) parentNode tick: 25
New target: (-2.529892;2.415452;-9.959358)
Current velocity 26.468
Current position (-3.433341;1.000000;-12.445374)
Distance: 13.1486
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.279595;0.000000)
Check point (0.000000;6.150684;-2.997229) parentNode tick: 26
New target: (-2.379655;2.416989;-9.545775)
Current velocity 26.4509
Current position (-3.282766;1.000000;-12.031038)
Distance: 12.6775
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.058762;0.000000)
Check point (0.000000;5.913340;-2.996473) parentNode tick: 27
New target: (-2.229142;2.416639;-9.131363)
Current velocity 26.4427
Current position (-3.132248;1.000000;-11.616827)
Distance: 12.2066
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.829595;0.000000)
Check point (0.000000;5.667054;-2.995593) parentNode tick: 28
New target: (-2.078253;2.414138;-8.715830)
Current velocity 26.4445
Current position (-2.981730;1.000000;-11.202583)
Distance: 11.7361
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.592095;0.000000)
Check point (0.000000;5.411829;-2.994579) parentNode tick: 29
New target: (-1.926870;2.409155;-8.298836)
Current velocity 26.4579
Current position (-2.831150;1.000000;-10.788124)
Distance: 11.2665
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.346262;0.000000)
Check point (0.000000;5.147668;-2.993420) parentNode tick: 30
New target: (-1.774857;2.401281;-7.879986)
Current velocity 26.4845
Current position (-2.680438;1.000000;-10.373244)
Distance: 10.7982
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.092095;0.000000)
Check point (0.000000;4.874577;-2.992104) parentNode tick: 31
New target: (-1.622055;2.389993;-7.458818)
Current velocity 26.5262
Current position (-2.529509;1.000000;-9.957707)
Distance: 10.3318
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.115371;0.000000)
Check point (0.000000;4.899584;-2.992229) parentNode tick: 32
New target: (-1.484689;2.465180;-7.080596)
Current velocity 26.5856
Current position (-2.378271;1.000000;-9.541238)
Distance: 9.89623
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.288804;0.000000)
Check point (0.000000;5.085930;-2.993132) parentNode tick: 33
New target: (-1.360890;2.591392;-6.740042)
Current velocity 26.1787
Current position (-2.229070;1.000000;-9.130384)
Distance: 9.4865
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.453904;0.000000)
Check point (0.000000;5.263332;-2.993941) parentNode tick: 34
New target: (-1.243793;2.718171;-6.417960)
Current velocity 25.4312
Current position (-2.083445;1.000000;-8.729425)
Distance: 9.09161
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.610671;0.000000)
Check point (0.000000;5.431790;-2.994662) parentNode tick: 35
New target: (-1.134242;2.843806;-6.116669)
Current velocity 24.5925
Current position (-1.942330;1.000000;-8.340938)
Distance: 8.71425
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.759104;0.000000)
Check point (0.000000;5.591300;-2.995303) parentNode tick: 36
New target: (-1.032538;2.966641;-5.836995)
Current velocity 23.6651
Current position (-1.806212;1.000000;-7.966256)
Distance: 8.35572
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.899204;0.000000)
Check point (0.000000;5.741861;-2.995871) parentNode tick: 37
New target: (-0.938822;3.084973;-5.579317)
Current velocity 22.6546
Current position (-1.675556;1.000000;-7.606658)
Distance: 8.01721
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.030971;0.000000)
Check point (0.000000;5.883472;-2.996372) parentNode tick: 38
New target: (-0.853061;3.197174;-5.343537)
Current velocity 21.5705
Current position (-1.550795;1.000000;-7.263328)
Distance: 7.69972
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.154404;0.000000)
Check point (0.000000;6.016131;-2.996812) parentNode tick: 39
New target: (-0.775047;3.301814;-5.129080)
Current velocity 20.4265
Current position (-1.432308;1.000000;-6.937300)
Distance: 7.40399
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.269504;0.000000)
Check point (0.000000;6.139838;-2.997196) parentNode tick: 40
New target: (-0.704418;3.397786;-4.934942)
Current velocity 19.2397
Current position (-1.320396;1.000000;-6.629402)
Distance: 7.13043
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.376271;0.000000)
Check point (0.000000;6.254591;-2.997531) parentNode tick: 41
New target: (-0.640688;3.484377;-4.759783)
Current velocity 18.0295
Current position (-1.215269;1.000000;-6.340203)
Distance: 6.87908
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.474704;0.000000)
Check point (0.000000;6.360390;-2.997821) parentNode tick: 42
New target: (-0.583290;3.561304;-4.602038)
Current velocity 16.8163
Current position (-1.117029;1.000000;-6.069977)
Distance: 6.64955
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.564804;0.000000)
Check point (0.000000;6.457234;-2.998071) parentNode tick: 43
New target: (-0.531614;3.628683;-4.460029)
Current velocity 15.6196
Current position (-1.025663;1.000000;-5.818684)
Distance: 6.44106
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.646571;0.000000)
Check point (0.000000;6.545122;-2.998284) parentNode tick: 44
New target: (-0.485049;3.686967;-4.332075)
Current velocity 14.4569
Current position (-0.941047;1.000000;-5.585979)
Distance: 6.25244
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.720004;0.000000)
Check point (0.000000;6.624053;-2.998465) parentNode tick: 45
New target: (-0.443014;3.736854;-4.216570)
Current velocity 13.3424
Current position (-0.862959;1.000000;-5.371245)
Distance: 6.08222
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.785104;0.000000)
Check point (0.000000;6.694029;-2.998617) parentNode tick: 46
New target: (-0.404970;3.779191;-4.112038)
Current velocity 12.2867
Current position (-0.791095;1.000000;-5.173643)
Distance: 5.92868
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.841871;0.000000)
Check point (0.000000;6.755047;-2.998743) parentNode tick: 47
New target: (-0.370440;3.814881;-4.017159)
Current velocity 11.2965
Current position (-0.725094;1.000000;-4.992178)
Distance: 5.78998
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.890304;0.000000)
Check point (0.000000;6.807109;-2.998846) parentNode tick: 48
New target: (-0.339002;3.844824;-3.930779)
Current velocity 10.3752
Current position (-0.664562;1.000000;-4.825759)
Distance: 5.66423
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.930404;0.000000)
Check point (0.000000;6.850213;-2.998928) parentNode tick: 49
New target: (-0.310295;3.869861;-3.851902)
Current velocity 9.52354
Current position (-0.609087;1.000000;-4.673254)
Distance: 5.54955
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.962171;0.000000)
Check point (0.000000;6.884360;-2.998991) parentNode tick: 50
New target: (-0.284010;3.890757;-3.779675)
Current velocity 8.74011
Current position (-0.558262;1.000000;-4.533539)
Distance: 5.44418
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.985604;0.000000)
Check point (0.000000;6.909549;-2.999036) parentNode tick: 51
New target: (-0.259881;3.908180;-3.713371)
Current velocity 8.022
Current position (-0.511693;1.000000;-4.405528)
Distance: 5.34643
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.000704;0.000000)
Check point (0.000000;6.925780;-2.999064) parentNode tick: 52
New target: (-0.237683;3.922707;-3.652371)
Current velocity 7.36539
Current position (-0.469006;1.000000;-4.288194)
Distance: 5.25479
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.007471;0.000000)
Check point (0.000000;6.933054;-2.999077) parentNode tick: 53
New target: (-0.217224;3.934828;-3.596142)
Current velocity 6.76595
Current position (-0.429855;1.000000;-4.180582)
Distance: 5.16787
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.005904;0.000000)
Check point (0.000000;6.931370;-2.999074) parentNode tick: 54
New target: (-0.198337;3.944953;-3.544228)
Current velocity 6.21918
Current position (-0.393921;1.000000;-4.081813)
Distance: 5.08447
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.996004;0.000000)
Check point (0.000000;6.920728;-2.999055) parentNode tick: 55
New target: (-0.180880;3.953423;-3.496234)
Current velocity 5.72058
Current position (-0.360913;1.000000;-3.991086)
Distance: 5.00351
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.977771;0.000000)
Check point (0.000000;6.901128;-2.999021) parentNode tick: 56
New target: (-0.164726;3.960522;-3.451814)
Current velocity 5.26584
Current position (-0.330567;1.000000;-3.907675)
Distance: 4.92405
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.951204;0.000000)
Check point (0.000000;6.872571;-2.998969) parentNode tick: 57
New target: (-0.149766;3.966482;-3.410667)
Current velocity 4.85089
Current position (-0.302644;1.000000;-3.830921)
Distance: 4.84532
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.916304;0.000000)
Check point (0.000000;6.835057;-2.998900) parentNode tick: 58
New target: (-0.135903;3.971497;-3.372525)
Current velocity 4.47197
Current position (-0.276929;1.000000;-3.760234)
Distance: 4.76664
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.873071;0.000000)
Check point (0.000000;6.788584;-2.998810) parentNode tick: 59
New target: (-0.123052;3.975725;-3.337150)
Current velocity 4.12561
Current position (-0.253229;1.000000;-3.695079)
Distance: 4.68746
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.821504;0.000000)
Check point (0.000000;6.733155;-2.998699) parentNode tick: 60
New target: (-0.111136;3.979298;-3.304328)
Current velocity 3.80867
Current position (-0.231370;1.000000;-3.634977)
Distance: 4.60733
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.761604;0.000000)
Check point (0.000000;6.668769;-2.998563) parentNode tick: 61
New target: (-0.100086;3.982322;-3.273869)
Current velocity 3.51831
Current position (-0.211195;1.000000;-3.579495)
Distance: 4.52591
Target velocity 30

bounty < 10
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.079595;0.000000)
Check point (0.000000;9.160536;-2.998908) parentNode tick: 2
New target: (-4.934748;2.288691;-16.576049)
Current velocity 0
Current position (-5.860171;1.000000;-19.122196)
Distance: 20.635
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.058762;0.000000)
Check point (0.000000;9.138141;-2.998950) parentNode tick: 3
New target: (-4.929625;2.286666;-16.561961)
Current velocity 1.66667
Current position (-5.855379;1.000000;-19.109012)
Distance: 20.6163
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;6.029595;0.000000)
Check point (0.000000;9.106790;-2.999007) parentNode tick: 4
New target: (-4.915101;2.285181;-16.522009)
Current velocity 3.33333
Current position (-5.841099;1.000000;-19.069722)
Distance: 20.5687
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.992095;0.000000)
Check point (0.000000;9.066480;-2.999078) parentNode tick: 5
New target: (-4.891175;2.284233;-16.456191)
Current velocity 5
Current position (-5.817329;1.000000;-19.004324)
Distance: 20.4921
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.946262;0.000000)
Check point (0.000000;9.017212;-2.999161) parentNode tick: 6
New target: (-4.857846;2.283826;-16.364506)
Current velocity 6.66667
Current position (-5.784071;1.000000;-18.912820)
Distance: 20.3867
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.892095;0.000000)
Check point (0.000000;8.958986;-2.999254) parentNode tick: 7
New target: (-4.815117;2.283966;-16.246960)
Current velocity 8.33333
Current position (-5.741323;1.000000;-18.795208)
Distance: 20.2523
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.829595;0.000000)
Check point (0.000000;8.891802;-2.999355) parentNode tick: 8
New target: (-4.762989;2.284668;-16.103556)
Current velocity 10
Current position (-5.689087;1.000000;-18.651490)
Distance: 20.089
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.758762;0.000000)
Check point (0.000000;8.815659;-2.999460) parentNode tick: 9
New target: (-4.701465;2.285948;-15.934304)
Current velocity 11.6667
Current position (-5.627362;1.000000;-18.481665)
Distance: 19.8969
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.679595;0.000000)
Check point (0.000000;8.730558;-2.999567) parentNode tick: 10
New target: (-4.630550;2.287832;-15.739215)
Current velocity 13.3333
Current position (-5.556147;1.000000;-18.285734)
Distance: 19.6758
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.592095;0.000000)
Check point (0.000000;8.636498;-2.999671) parentNode tick: 11
New target: (-4.550250;2.290350;-15.518305)
Current velocity 15
Current position (-5.475443;1.000000;-18.063696)
Distance: 19.4259
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.496262;0.000000)
Check point (0.000000;8.533479;-2.999769) parentNode tick: 12
New target: (-4.460571;2.293543;-15.271594)
Current velocity 16.6667
Current position (-5.385250;1.000000;-17.815551)
Distance: 19.1471
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.392095;0.000000)
Check point (0.000000;8.421501;-2.999856) parentNode tick: 13
New target: (-4.361523;2.297457;-14.999106)
Current velocity 18.3333
Current position (-5.285567;1.000000;-17.541299)
Distance: 18.8395
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.279595;0.000000)
Check point (0.000000;8.300565;-2.999927) parentNode tick: 14
New target: (-4.253117;2.302151;-14.700871)
Current velocity 20
Current position (-5.176395;1.000000;-17.240942)
Distance: 18.503
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.158762;0.000000)
Check point (0.000000;8.170669;-2.999976) parentNode tick: 15
New target: (-4.135367;2.307697;-14.376925)
Current velocity 21.6667
Current position (-5.057733;1.000000;-16.914477)
Distance: 18.1377
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;5.029595;0.000000)
Check point (0.000000;8.031815;-2.999999) parentNode tick: 16
New target: (-4.008289;2.314179;-14.027315)
Current velocity 23.3333
Current position (-4.929580;1.000000;-16.561906)
Distance: 17.7436
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.892095;0.000000)
Check point (0.000000;7.884003;-2.999989) parentNode tick: 17
New target: (-3.871903;2.321703;-13.652096)
Current velocity 25
Current position (-4.791938;1.000000;-16.183229)
Distance: 17.3207
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.746262;0.000000)
Check point (0.000000;7.727232;-2.999940) parentNode tick: 18
New target: (-3.726236;2.330394;-13.251336)
Current velocity 26.6667
Current position (-4.644805;1.000000;-15.778446)
Distance: 16.8691
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.592095;0.000000)
Check point (0.000000;7.561504;-2.999844) parentNode tick: 19
New target: (-3.574779;2.339546;-12.834638)
Current velocity 26.8888
Current position (-4.491788;1.000000;-15.357476)
Distance: 16.3991
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.429595;0.000000)
Check point (0.000000;7.386819;-2.999695) parentNode tick: 20
New target: (-3.423354;2.347735;-12.418010)
Current velocity 26.8433
Current position (-4.338951;1.000000;-14.936996)
Distance: 15.928
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.258762;0.000000)
Check point (0.000000;7.203179;-2.999485) parentNode tick: 21
New target: (-3.271995;2.354857;-12.001546)
Current velocity 26.8023
Current position (-4.186349;1.000000;-14.517160)
Distance: 15.4562
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.079595;0.000000)
Check point (0.000000;7.010583;-2.999206) parentNode tick: 22
New target: (-3.120654;2.360816;-11.585108)
Current velocity 26.7663
Current position (-4.033956;1.000000;-14.097888)
Distance: 14.9836
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.892095;0.000000)
Check point (0.000000;6.809034;-2.998850) parentNode tick: 23
New target: (-2.969279;2.365498;-11.168544)
Current velocity 26.7361
Current position (-3.881738;1.000000;-13.679089)
Distance: 14.5103
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.696262;0.000000)
Check point (0.000000;6.598534;-2.998408) parentNode tick: 24
New target: (-2.817809;2.368767;-10.751679)
Current velocity 26.7122
Current position (-3.729661;1.000000;-13.260664)
Distance: 14.0366
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.492095;0.000000)
Check point (0.000000;6.379083;-2.997871) parentNode tick: 25
New target: (-2.666175;2.370465;-10.334318)
Current velocity 26.6955
Current position (-3.577686;1.000000;-12.842500)
Distance: 13.5625
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.279595;0.000000)
Check point (0.000000;6.150684;-2.997229) parentNode tick: 26
New target: (-2.514300;2.370402;-9.916238)
Current velocity 26.6868
Current position (-3.425767;1.000000;-12.424469)
Distance: 13.0882
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.058762;0.000000)
Check point (0.000000;5.913340;-2.996473) parentNode tick: 27
New target: (-2.362097;2.368350;-9.497185)
Current velocity 26.6871
Current position (-3.273856;1.000000;-12.006431)
Distance: 12.6139
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.829595;0.000000)
Check point (0.000000;5.667054;-2.995593) parentNode tick: 28
New target: (-2.209465;2.364031;-9.076871)
Current velocity 26.6976
Current position (-3.121896;1.000000;-11.588223)
Distance: 12.14
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.592095;0.000000)
Check point (0.000000;5.411829;-2.994579) parentNode tick: 29
New target: (-2.056290;2.357103;-8.654963)
Current velocity 26.7197
Current position (-2.969825;1.000000;-11.169665)
Distance: 11.6669
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.346262;0.000000)
Check point (0.000000;5.147668;-2.993420) parentNode tick: 30
New target: (-1.902439;2.347140;-8.231078)
Current velocity 26.7549
Current position (-2.817570;1.000000;-10.750552)
Distance: 11.1949
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.092095;0.000000)
Check point (0.000000;4.874577;-2.992104) parentNode tick: 31
New target: (-1.747757;2.333605;-7.804767)
Current velocity 26.8052
Current position (-2.665051;1.000000;-10.330647)
Distance: 10.7246
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.115371;0.000000)
Check point (0.000000;4.899584;-2.992229) parentNode tick: 32
New target: (-1.607299;2.404614;-7.418045)
Current velocity 26.8728
Current position (-2.512174;1.000000;-9.909684)
Distance: 10.2838
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.288804;0.000000)
Check point (0.000000;5.085930;-2.993132) parentNode tick: 33
New target: (-1.479351;2.525934;-7.066067)
Current velocity 26.5086
Current position (-2.361146;1.000000;-9.493815)
Distance: 9.86755
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.453904;0.000000)
Check point (0.000000;5.263332;-2.993941) parentNode tick: 34
New target: (-1.357599;2.648416;-6.731166)
Current velocity 25.8293
Current position (-2.213415;1.000000;-9.087074)
Distance: 9.46509
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.610671;0.000000)
Check point (0.000000;5.431790;-2.994662) parentNode tick: 35
New target: (-1.242851;2.770647;-6.415567)
Current velocity 25.0654
Current position (-2.069808;1.000000;-8.691738)
Distance: 9.0788
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.759104;0.000000)
Check point (0.000000;5.591300;-2.995303) parentNode tick: 36
New target: (-1.135472;2.891206;-6.120267)
Current velocity 24.2174
Current position (-1.930783;1.000000;-8.309062)
Distance: 8.70993
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.899204;0.000000)
Check point (0.000000;5.741861;-2.995871) parentNode tick: 37
New target: (-1.035698;3.008574;-5.845908)
Current velocity 23.2881
Current position (-1.796787;1.000000;-7.940275)
Distance: 8.35963
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.030971;0.000000)
Check point (0.000000;5.883472;-2.996372) parentNode tick: 38
New target: (-0.943616;3.121220;-5.592727)
Current velocity 22.2837
Current position (-1.668248;1.000000;-7.586550)
Distance: 8.02893
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.154404;0.000000)
Check point (0.000000;6.016131;-2.996812) parentNode tick: 39
New target: (-0.859159;3.227717;-5.360532)
Current velocity 21.2142
Current position (-1.545557;1.000000;-7.248953)
Distance: 7.71865
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.269504;0.000000)
Check point (0.000000;6.139838;-2.997196) parentNode tick: 40
New target: (-0.782106;3.326843;-5.148712)
Current velocity 20.093
Current position (-1.429046;1.000000;-6.928395)
Distance: 7.42937
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.376271;0.000000)
Check point (0.000000;6.254591;-2.997531) parentNode tick: 41
New target: (-0.712103;3.417678;-4.956288)
Current velocity 18.9362
Current position (-1.318973;1.000000;-6.625582)
Distance: 7.16133
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.474704;0.000000)
Check point (0.000000;6.360390;-2.997821) parentNode tick: 42
New target: (-0.648692;3.499659;-4.781995)
Current velocity 17.7619
Current position (-1.215509;1.000000;-6.340977)
Distance: 6.91445
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.564804;0.000000)
Check point (0.000000;6.457234;-2.998071) parentNode tick: 43
New target: (-0.591345;3.572597;-4.624379)
Current velocity 16.5883
Current position (-1.118722;1.000000;-6.074765)
Distance: 6.68824
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.646571;0.000000)
Check point (0.000000;6.545122;-2.998284) parentNode tick: 44
New target: (-0.539501;3.636646;-4.481898)
Current velocity 15.4329
Current position (-1.028580;1.000000;-5.826851)
Distance: 6.48186
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.720004;0.000000)
Check point (0.000000;6.624053;-2.998465) parentNode tick: 45
New target: (-0.492601;3.692247;-4.353006)
Current velocity 14.3112
Current position (-0.944951;1.000000;-5.596866)
Distance: 6.29415
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.785104;0.000000)
Check point (0.000000;6.694029;-2.998617) parentNode tick: 46
New target: (-0.450107;3.740043;-4.236227)
Current velocity 13.2356
Current position (-0.867615;1.000000;-5.384203)
Distance: 6.12366
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.841871;0.000000)
Check point (0.000000;6.755047;-2.998743) parentNode tick: 47
New target: (-0.411523;3.780806;-4.130196)
Current velocity 12.2154
Current position (-0.796282;1.000000;-5.188063)
Distance: 5.96878
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.890304;0.000000)
Check point (0.000000;6.807109;-2.998846) parentNode tick: 48
New target: (-0.376403;3.815352;-4.033685)
Current velocity 11.2566
Current position (-0.730612;1.000000;-5.007507)
Distance: 5.82776
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.930404;0.000000)
Check point (0.000000;6.850213;-2.998928) parentNode tick: 49
New target: (-0.344354;3.844496;-3.945611)
Current velocity 10.3624
Current position (-0.670237;1.000000;-4.841515)
Distance: 5.69884
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.962171;0.000000)
Check point (0.000000;6.884360;-2.998991) parentNode tick: 50
New target: (-0.315031;3.869002;-3.865029)
Current velocity 9.53333
Current position (-0.614772;1.000000;-4.689032)
Distance: 5.58027
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.985604;0.000000)
Check point (0.000000;6.909549;-2.999036) parentNode tick: 51
New target: (-0.288140;3.889568;-3.791124)
Current velocity 8.76828
Current position (-0.563837;1.000000;-4.549008)
Distance: 5.47039
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.000704;0.000000)
Check point (0.000000;6.925780;-2.999064) parentNode tick: 52
New target: (-0.263424;3.906809;-3.723195)
Current velocity 8.06472
Current position (-0.517062;1.000000;-4.420423)
Distance: 5.36765
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.007471;0.000000)
Check point (0.000000;6.933054;-2.999077) parentNode tick: 53
New target: (-0.240665;3.921261;-3.660637)
Current velocity 7.41929
Current position (-0.474095;1.000000;-4.302311)
Distance: 5.27063
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;4.005904;0.000000)
Check point (0.000000;6.931370;-2.999074) parentNode tick: 54
New target: (-0.219672;3.933378;-3.602929)
Current velocity 6.82814
Current position (-0.434610;1.000000;-4.193769)
Distance: 5.17803
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.996004;0.000000)
Check point (0.000000;6.920728;-2.999055) parentNode tick: 55
New target: (-0.200282;3.943548;-3.549619)
Current velocity 6.28722
Current position (-0.398302;1.000000;-4.093963)
Distance: 5.08873
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.977771;0.000000)
Check point (0.000000;6.901128;-2.999021) parentNode tick: 56
New target: (-0.182352;3.952094;-3.500314)
Current velocity 5.79243
Current position (-0.364894;1.000000;-4.002127)
Distance: 5.00173
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.951204;0.000000)
Check point (0.000000;6.872571;-2.998969) parentNode tick: 57
New target: (-0.165758;3.959287;-3.454669)
Current velocity 5.33983
Current position (-0.334133;1.000000;-3.917565)
Distance: 4.91615
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.916304;0.000000)
Check point (0.000000;6.835057;-2.998900) parentNode tick: 58
New target: (-0.150388;3.965350;-3.412380)
Current velocity 4.92567
Current position (-0.305789;1.000000;-3.839642)
Distance: 4.83127
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.873071;0.000000)
Check point (0.000000;6.788584;-2.998810) parentNode tick: 59
New target: (-0.136147;3.970470;-3.373178)
Current velocity 4.54644
Current position (-0.279654;1.000000;-3.767785)
Distance: 4.74647
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.821504;0.000000)
Check point (0.000000;6.733155;-2.998699) parentNode tick: 60
New target: (-0.122946;3.974802;-3.336822)
Current velocity 4.19892
Current position (-0.255539;1.000000;-3.701477)
Distance: 4.66124
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.761604;0.000000)
Check point (0.000000;6.668769;-2.998563) parentNode tick: 61
New target: (-0.110708;3.978474;-3.303095)
Current velocity 3.88014
Current position (-0.233275;1.000000;-3.640248)
Distance: 4.57518
Target velocity 30

move robot 1  on tick 1
move robot 2  on tick 1
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.761604;0.000000)
Check point (0.000000;6.668769;-2.998563) parentNode tick: 61
New target: (-0.100086;3.982322;-3.273869)
Current velocity 3.51831
Current position (-0.211195;1.000000;-3.579495)
Distance: 4.52591
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.693371;0.000000)
Check point (0.000000;6.595426;-2.998401) parentNode tick: 62
New target: (-0.089840;3.984888;-3.245599)
Current velocity 3.25197
Current position (-0.192563;1.000000;-3.528245)
Distance: 4.44295
Target velocity 30

move robot 2  on tick 2
move robot 1  on tick 2
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.693371;0.000000)
Check point (0.000000;6.595426;-2.998401) parentNode tick: 62
New target: (-0.089840;3.984888;-3.245599)
Current velocity 3.25197
Current position (-0.192563;1.000000;-3.528245)
Distance: 4.44295
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.616804;0.000000)
Check point (0.000000;6.513126;-2.998208) parentNode tick: 63
New target: (-0.080342;3.987070;-3.219361)
Current velocity 3.00734
Current position (-0.175347;1.000000;-3.480875)
Distance: 4.35831
Target velocity 30

move robot 1  on tick 3
move robot 2  on tick 3
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.616804;0.000000)
Check point (0.000000;6.513126;-2.998208) parentNode tick: 63
New target: (-0.080342;3.987070;-3.219361)
Current velocity 3.00734
Current position (-0.175347;1.000000;-3.480875)
Distance: 4.35831
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.531904;0.000000)
Check point (0.000000;6.421871;-2.997981) parentNode tick: 64
New target: (-0.071542;3.988927;-3.195012)
Current velocity 2.78236
Current position (-0.159432;1.000000;-3.437069)
Distance: 4.27193
Target velocity 30

move robot 2  on tick 4
move robot 1  on tick 4
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.531904;0.000000)
Check point (0.000000;6.421871;-2.997981) parentNode tick: 64
New target: (-0.071542;3.988927;-3.195012)
Current velocity 2.78236
Current position (-0.159432;1.000000;-3.437069)
Distance: 4.27193
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.438671;0.000000)
Check point (0.000000;6.321660;-2.997717) parentNode tick: 65
New target: (-0.063393;3.990511;-3.172421)
Current velocity 2.57519
Current position (-0.144716;1.000000;-3.396540)
Distance: 4.18384
Target velocity 30

move robot 1  on tick 5
move robot 2  on tick 5
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.438671;0.000000)
Check point (0.000000;6.321660;-2.997717) parentNode tick: 65
New target: (-0.063393;3.990511;-3.172421)
Current velocity 2.57519
Current position (-0.144716;1.000000;-3.396540)
Distance: 4.18384
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.337104;0.000000)
Check point (0.000000;6.212494;-2.997411) parentNode tick: 66
New target: (-0.055853;3.991865;-3.151468)
Current velocity 2.38417
Current position (-0.131105;1.000000;-3.359031)
Distance: 4.09418
Target velocity 30

move robot 2  on tick 6
move robot 1  on tick 6
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.337104;0.000000)
Check point (0.000000;6.212494;-2.997411) parentNode tick: 66
New target: (-0.055853;3.991865;-3.151468)
Current velocity 2.38417
Current position (-0.131105;1.000000;-3.359031)
Distance: 4.09418
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.227204;0.000000)
Check point (0.000000;6.094375;-2.997058) parentNode tick: 67
New target: (-0.048885;3.993023;-3.132042)
Current velocity 2.20782
Current position (-0.118514;1.000000;-3.324304)
Distance: 4.00318
Target velocity 30

move robot 2  on tick 7
move robot 1  on tick 7
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.227204;0.000000)
Check point (0.000000;6.094375;-2.997058) parentNode tick: 67
New target: (-0.048885;3.993023;-3.132042)
Current velocity 2.20782
Current position (-0.118514;1.000000;-3.324304)
Distance: 4.00318
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.108971;0.000000)
Check point (0.000000;5.967302;-2.996653) parentNode tick: 68
New target: (-0.042454;3.994015;-3.114040)
Current velocity 2.04482
Current position (-0.106868;1.000000;-3.292149)
Distance: 3.91119
Target velocity 30

move robot 2  on tick 8
move robot 1  on tick 8
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;3.108971;0.000000)
Check point (0.000000;5.967302;-2.996653) parentNode tick: 68
New target: (-0.042454;3.994015;-3.114040)
Current velocity 2.04482
Current position (-0.106868;1.000000;-3.292149)
Distance: 3.91119
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.982404;0.000000)
Check point (0.000000;5.831276;-2.996191) parentNode tick: 69
New target: (-0.036527;3.994867;-3.097367)
Current velocity 1.89398
Current position (-0.096097;1.000000;-3.262369)
Distance: 3.81867
Target velocity 30

move robot 2  on tick 9
move robot 1  on tick 9
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.982404;0.000000)
Check point (0.000000;5.831276;-2.996191) parentNode tick: 69
New target: (-0.036527;3.994867;-3.097367)
Current velocity 1.89398
Current position (-0.096097;1.000000;-3.262369)
Distance: 3.81867
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.847504;0.000000)
Check point (0.000000;5.686300;-2.995666) parentNode tick: 70
New target: (-0.031077;3.995597;-3.081935)
Current velocity 1.75425
Current position (-0.086139;1.000000;-3.234788)
Distance: 3.7262
Target velocity 30

move robot 2  on tick 10
move robot 1  on tick 10
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.847504;0.000000)
Check point (0.000000;5.686300;-2.995666) parentNode tick: 70
New target: (-0.031077;3.995597;-3.081935)
Current velocity 1.75425
Current position (-0.086139;1.000000;-3.234788)
Distance: 3.7262
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.704271;0.000000)
Check point (0.000000;5.532374;-2.995071) parentNode tick: 71
New target: (-0.026076;3.996226;-3.067660)
Current velocity 1.62468
Current position (-0.076937;1.000000;-3.209243)
Distance: 3.63452
Target velocity 30

move robot 2  on tick 11
move robot 1  on tick 11
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.704271;0.000000)
Check point (0.000000;5.532374;-2.995071) parentNode tick: 71
New target: (-0.026076;3.996226;-3.067660)
Current velocity 1.62468
Current position (-0.076937;1.000000;-3.209243)
Distance: 3.63452
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.552704;0.000000)
Check point (0.000000;5.369499;-2.994401) parentNode tick: 72
New target: (-0.021501;3.996766;-3.054465)
Current velocity 1.50441
Current position (-0.068438;1.000000;-3.185588)
Distance: 3.54451
Target velocity 30

move robot 1  on tick 12
move robot 2  on tick 12
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.552704;0.000000)
Check point (0.000000;5.369499;-2.994401) parentNode tick: 72
New target: (-0.021501;3.996766;-3.054465)
Current velocity 1.50441
Current position (-0.068438;1.000000;-3.185588)
Distance: 3.54451
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.392804;0.000000)
Check point (0.000000;5.197678;-2.993648) parentNode tick: 73
New target: (-0.017329;3.997230;-3.042274)
Current velocity 1.39271
Current position (-0.060596;1.000000;-3.163684)
Distance: 3.45723
Target velocity 30

move robot 2  on tick 13
move robot 1  on tick 13
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.392804;0.000000)
Check point (0.000000;5.197678;-2.993648) parentNode tick: 73
New target: (-0.017329;3.997230;-3.042274)
Current velocity 1.39271
Current position (-0.060596;1.000000;-3.163684)
Distance: 3.45723
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.224571;0.000000)
Check point (0.000000;5.016913;-2.992804) parentNode tick: 74
New target: (-0.013542;3.997630;-3.031019)
Current velocity 1.28889
Current position (-0.053369;1.000000;-3.143406)
Distance: 3.37393
Target velocity 30

move robot 1  on tick 14
move robot 2  on tick 14
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.224571;0.000000)
Check point (0.000000;5.016913;-2.992804) parentNode tick: 74
New target: (-0.013542;3.997630;-3.031019)
Current velocity 1.28889
Current position (-0.053369;1.000000;-3.143406)
Distance: 3.37393
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.048004;0.000000)
Check point (0.000000;4.827205;-2.991864) parentNode tick: 75
New target: (-0.010122;3.997973;-3.020632)
Current velocity 1.19235
Current position (-0.046717;1.000000;-3.124639)
Distance: 3.29604
Target velocity 30

move robot 2  on tick 15
move robot 1  on tick 15
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.048004;0.000000)
Check point (0.000000;4.827205;-2.991864) parentNode tick: 75
New target: (-0.010122;3.997973;-3.020632)
Current velocity 1.19235
Current position (-0.046717;1.000000;-3.124639)
Distance: 3.29604
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.091284;0.000000)
Check point (0.000000;4.873705;-2.992099) parentNode tick: 76
New target: (-0.009174;3.998510;-3.018121)
Current velocity 1.10257
Current position (-0.040606;1.000000;-3.107273)
Distance: 3.29358
Target velocity 30

move robot 1  on tick 16
move robot 2  on tick 16
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.091284;0.000000)
Check point (0.000000;4.873705;-2.992099) parentNode tick: 76
New target: (-0.009174;3.998510;-3.018121)
Current velocity 1.10257
Current position (-0.040606;1.000000;-3.107273)
Distance: 3.29358
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.209122;0.000000)
Check point (0.000000;5.000314;-2.992724) parentNode tick: 77
New target: (-0.008844;3.998954;-3.017653)
Current velocity 0.945311
Current position (-0.035331;1.000000;-3.092310)
Distance: 3.32048
Target velocity 30

move robot 1  on tick 17
move robot 2  on tick 17
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.209122;0.000000)
Check point (0.000000;5.000314;-2.992724) parentNode tick: 77
New target: (-0.008844;3.998954;-3.017653)
Current velocity 0.945311
Current position (-0.035331;1.000000;-3.092310)
Distance: 3.32048
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.318627;0.000000)
Check point (0.000000;5.117975;-2.993282) parentNode tick: 78
New target: (-0.008390;3.999254;-3.016778)
Current velocity 0.792166
Current position (-0.030883;1.000000;-3.079768)
Distance: 3.35033
Target velocity 30

move robot 1  on tick 18
move robot 2  on tick 18
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.318627;0.000000)
Check point (0.000000;5.117975;-2.993282) parentNode tick: 78
New target: (-0.008390;3.999254;-3.016778)
Current velocity 0.792166
Current position (-0.030883;1.000000;-3.079768)
Distance: 3.35033
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.419799;0.000000)
Check point (0.000000;5.226685;-2.993778) parentNode tick: 79
New target: (-0.007872;3.999461;-3.015679)
Current velocity 0.668858
Current position (-0.027113;1.000000;-3.069207)
Distance: 3.3818
Target velocity 30

move robot 2  on tick 19
move robot 1  on tick 19
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.419799;0.000000)
Check point (0.000000;5.226685;-2.993778) parentNode tick: 79
New target: (-0.007872;3.999461;-3.015679)
Current velocity 0.668858
Current position (-0.027113;1.000000;-3.069207)
Distance: 3.3818
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.512637;0.000000)
Check point (0.000000;5.326445;-2.994216) parentNode tick: 80
New target: (-0.007327;3.999605;-3.014467)
Current velocity 0.56881
Current position (-0.023892;1.000000;-3.060246)
Distance: 3.41376
Target velocity 30

move robot 2  on tick 20
move robot 1  on tick 20
Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.512637;0.000000)
Check point (0.000000;5.326445;-2.994216) parentNode tick: 80
New target: (-0.007327;3.999605;-3.014467)
Current velocity 0.56881
Current position (-0.023892;1.000000;-3.060246)
Distance: 3.41376
Target velocity 30

Goal Target (0.000000;5.000000;40.000000)
Ball position (0.000000;2.597142;0.000000)
Check point (0.000000;5.417252;-2.994602) parentNode tick: 81
New target: (-0.006779;3.999707;-3.013211)
Current velocity 0.486844
Current position (-0.021123;1.000000;-3.052591)
Distance: 3.44523
Target velocity 30

move robot 2  on tick 21
move robot 1  on tick 21
*/

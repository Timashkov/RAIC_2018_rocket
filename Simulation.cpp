
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

const int DEPTH = 50;

void Simulation::init(const Game &g, const Rules &rul, const int &goal_keeper_id) {
    rules = rul;
    std::srand(rul.seed);

    this->goalKeeperId = goal_keeper_id;
    this->defaultGoalKeeperPosition = Vec3(0.0, 0.0, -(rules.arena.depth / 2.0));

    attackerId = -1;
    arena = rul.arena;
    engine = std::unique_ptr<SimulationEngine>(new SimulationEngine(rul));

    State st;
    setInitialState(g, st);
    baseNode = std::make_shared<TreeNode>(st, nullptr);

    inited = true;
    ENEMY_GOAL_TARGET = Vec3(0, arena.goal_height - rules.BALL_RADIUS, arena.depth / 2.0 + rul.BALL_RADIUS * 2);
    delta_time = 1.0 / rules.TICKS_PER_SECOND;
    delta_time = delta_time / rules.MICROTICKS_PER_TICK;
    GOAL_THRESHOLD = - rules.arena.depth / 4.0;
    ENEMY_GOAL_INACCESS_Z = rules.arena.depth / 2.0 - (rules.BALL_RADIUS * 2 + rules.arena.bottom_radius) ;
    ENEMY_GOAL_INACCESS_X = rules.arena.width / 2.0 - rules.arena.corner_radius;
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

 double radsum = rules.ROBOT_RADIUS  + rules.BALL_RADIUS;
    bool hitAchieved = false;
    
    vector<SimulationEntity> route;

    TreeNode *tn = baseNode.get();
    cout << "Check alternatives for node " << baseNode->state.current_tick << endl;
    while (tn->children.size() > 0) {
        route.clear();
        for (SimulationEntity &se: tn->state.robots) {
            if (!se.teammate)
                continue;
            
            if (se.touch &&
                se.position.getZ() - 1 < tn->state.ball.position.getZ() &&
                ((tn->state.ball.position.getZ() < 0 && se.id == goalKeeperId) ||
                 se.id != goalKeeperId)) {
        
                    Vec3 ballHitPosition = getHitPosition(tn->state.ball, se);
                    
                    cout << "Hit position " << ballHitPosition.toString() << " for tick " << tn->state.current_tick << endl;
                    
                    if (ballHitPosition == Vec3::None){
                        tn = tn->children[tn->children.size() - 1].get();
                        continue;
                    }
                    
                    Vec3 rp = se.position;
                    SimulationEntity ball = tn->state.ball;
  
                    
                    hitAchieved = checkAchievement(se, ballHitPosition, tn->state.ball.position,
                                                                tn->state.current_tick - current_tick, route);
                    
  
                    if (hitAchieved) {
                        cout << " For Attacker " << se.id << endl;
                        attackerId = se.id;
                        break;
                    }
                }
        }
        if (attackerId != -1) {
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

    cout << "ACTIONS " << tn->state.current_tick << endl;
    cout << " tn->state.current_tick "<< tn->state.current_tick << endl;
    
    if (hitAchieved) {
        cout<< " +++++++ TICK COLLISION EXISTS +++++++++"<<endl;
        int i = 0;
        while (tn->children.size() > 0 && i < route.size()) {
            TreeNode *childNode = tn->children[tn->children.size() - 1].get();
            for (SimulationEntity &robot: tn->state.robots) {
                if (robot.id == attackerId) {
                    //COPY FROM ROUTE
                    
                    robot.action = route.at(i).action;
                    i++;
                    cout << "Robot action target velocity "<< robot.action.target_velocity_x << ";"
                    << robot.action.target_velocity_y << ";" << robot.action.target_velocity_z << endl;
                    
                    
                } else if (robot.id == goalKeeperId) {
                    cout<< " GOALKEEPER id = "<< robot.id << endl;
                    Vec3 target = defaultGoalKeeperPosition;
                    
                    if (tn->state.ball.velocity.getZ() < 0){
                        double k = tn->state.ball.velocity.getZ() / tn->state.ball.velocity.getX();
                        double b = tn->state.ball.position.getZ() - k * tn->state.ball.position.getX();
                        double x = (-rules.arena.depth/2 - b ) / k;
                        if (abs(x) <= abs(rules.arena.goal_width /2)){
                            target.setX(x);
                        }
                    }
                    
                    if (tn->state.ball.position.getZ() < robot.position.getZ() + 1 ){
                        if (abs(target.getX() - robot.position.getX()) < radsum) {
                            target = robot.position;
                            if (abs(robot.position.getX()) < abs(tn->state.ball.position.getX()) + radsum) {
                                if (robot.position.getX() < tn->state.ball.position.getX())
                                    target.setX(target.getX() - radsum * 2);
                                else
                                    target.setX(target.getX() + radsum * 2);
                            } else {
                                target.setX(0.0);
                            }
                        }
                    }
                    
                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    Vec3 del = (target - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    if (del.len() > 1) {
                        del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    }
                    cout << "Target velocity " << del.toString() << " len " << del.len() << endl;
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
                    cout << endl;
                    moveRobotAndAdjustNextNode(robot, childNode);
                }
            }
            
            tn = childNode;
        }
        cout<< " ------- TICK COLLISION EXISTS END -------"<<endl;
        
    } else {
        cout << " +++++++++ FREE GOING +++++++++++"<<endl;
        TreeNode *targetBallNode = tn;
        for (int j = 0; j < 15; j++) {
            targetBallNode = targetBallNode->children[tn->children.size() - 1].get();
        }
        for (int i = 0; i < 15; i++) {
            TreeNode *childNode = tn->children[tn->children.size() - 1].get();
            for (SimulationEntity &robot: tn->state.robots) {
                Vec3 target = defaultGoalKeeperPosition; // FIND BALL x GOAL position
                
                if (robot.id == goalKeeperId) {
                    
                    if (tn->state.ball.velocity.getZ() < 0){
                        double k = tn->state.ball.velocity.getZ() / tn->state.ball.velocity.getX();
                        double b = tn->state.ball.position.getZ() - k * tn->state.ball.position.getX();
                        double x = (-rules.arena.depth/2 - b ) / k;
                        if (abs(x) <= abs(rules.arena.goal_width /2)){
                            target.setX(x);
                        }
                    }
                    
                    if (tn->state.ball.position.getZ() < robot.position.getZ() + rules.BALL_RADIUS ){
                        if (abs(target.getX() - robot.position.getX()) < radsum) {
                            target = robot.position;
                            if (abs(robot.position.getX()) < abs(targetBallNode->state.ball.position.getX()) + radsum) {
                                if (robot.position.getX() < targetBallNode->state.ball.position.getX())
                                    target.setX(target.getX() - radsum * 2);
                                else
                                    target.setX(target.getX() + radsum * 2);
                            } else {
                                target.setX(0.0);
                            }
                        }
                    }
                    
                } else {

                    target = targetBallNode->state.ball.position;
                    cout << " TArgt ball node position "<< target.toString()<< endl;
                    cout<< " Robot current speed " << robot.velocity.toString() << endl;
                    cout << " ball current pos "<< tn->state.ball.position.toString() << endl;
                    target.setY(1.0);
                    
                    target.setZ(target.getZ() - rules.BALL_RADIUS * 2);
                    
                    cout << " Default--- Go 2 ball LAST ATTACKER " << target.toString() << " parentNode tick: "
                    << tn->state.current_tick << " robot id "<< robot.id
                    << endl;
                    
                    
                    
                    if (tn->state.ball.position.getZ() < robot.position.getZ() + 1|| target.getZ() < robot.position.getZ() + 1){
                        if (abs(target.getX() - robot.position.getX()) < radsum) {
                            target = robot.position;
                            if (abs(robot.position.getX()) < abs(targetBallNode->state.ball.position.getX()) + radsum) {
                                if (robot.position.getX() < targetBallNode->state.ball.position.getX())
                                    target.setX(target.getX() - radsum * 2);
                                else
                                    target.setX(target.getX() + radsum * 2);
                            } else {
                                target.setX(0.0);
                            }
                        }
                    }
                }
                
                cout << " Default--- Go 2 ball LAST ATTACKER " << target.toString() << " parentNode tick: "
                << tn->state.current_tick << " robot id "<< robot.id
                << endl;

                robot.action.jump_speed = 0.0;
                robot.action.use_nitro = false;
                Vec3 del = target - robot.position;
                if (del.len() > 1) {
                    del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                }
                robot.action.target_velocity_x = del.getX();
                robot.action.target_velocity_y = del.getY();
                robot.action.target_velocity_z = del.getZ();
                robot.action_set = true;
                moveRobotAndAdjustNextNode(robot, childNode);
            }
            tn = childNode;
        }
        cout<< " ------ FREEE GOING -----"<<endl;
    }

    cout << " Finalization " << tn->state.current_tick << endl;
    for (SimulationEntity &se :tn->state.robots) {
        se.action_set = false;
    }
    tn = tn->parent;
    cout << "ACTIONS DONE on tick " << tn->state.current_tick << endl;
}

bool Simulation::checkAchievement(SimulationEntity rr1, Vec3 bptarget, Vec3 ballPosition, int max_attempts, vector<SimulationEntity>& route) {
    
    
    cout << "## Check achievement for robot on position"<< rr1.position.toString() <<endl<< "and bp target "<< bptarget.toString() <<endl;
    cout << "Available delta time "<<max_attempts<<endl;
    
    Vec3 bptarget_plain = Vec3(bptarget.getX(), 1, bptarget.getZ());
    
    Vec3 initial_delta = bptarget_plain - rr1.position;
    double initial_delta_length = initial_delta.len();
    cout << "Initial delta length "<< initial_delta_length << endl;
    if (delta_time * rules.MICROTICKS_PER_TICK * rules.ROBOT_MAX_GROUND_SPEED * max_attempts < initial_delta_length) {
        cout << " ->Can not be achieved , initial len "<<initial_delta_length << endl;
        cout << " ->Available len "<<delta_time * rules.MICROTICKS_PER_TICK * rules.ROBOT_MAX_GROUND_SPEED * max_attempts<<endl;
        cout << " ##->Return FALSE ##"<<endl;
        return false;
    }
    
    //TODO: find correct tv
    Vec3 tv = initial_delta.normalized();
    tv.mulAndApply(rules.ROBOT_MAX_GROUND_SPEED);
    JumpParams jp = getJumpParams(bptarget, false);
    if (jp.jump_ticks == -1){
        jp = getJumpParams(bptarget, true);
        if (jp.jump_ticks == -1){
            cout << "can not jump so high"<<endl;
            return false;
        }
    }
    max_attempts -= jp.jump_ticks;
    if (max_attempts < 0){
        // not enough time for jump
        cout << "Not enough time for jump"<<endl;
        return false;
    }
    
    cout << "Jump ticks "<< jp.jump_ticks<<endl;
    cout << "Full jump time in sec "<< jp.fullTimeSec <<endl;
    
    double t = ((double)max_attempts) / rules.TICKS_PER_SECOND;
    Vec3 acceleration = (initial_delta - rr1.velocity * (t + jp.fullTimeSec)) * 2 / (t*t + 2*t*(jp.fullTimeSec));
    Vec3 Vfinal = rr1.velocity + acceleration * t;
    Vec3 path = rr1.velocity * t + acceleration * t*t /2.0 + Vfinal * jp.fullTimeSec;
    
    cout << "Acceleration "<< acceleration.toString() <<endl;
    cout << "Path len " << path.len() <<endl;
    cout << "Velocity final "<< Vfinal.toString()<< endl;
    cout << "Time "<< t << " equals to "<< t * rules.TICKS_PER_SECOND << endl;
    
    double tvmaxk = 0;
    
    if ( path.len() < initial_delta_length - EPS){
        // not enough time for velocity change
        cout << "not enough time for velocity change"<<endl;
        return false;
    }
    
    if (acceleration.len() > rules.ROBOT_ACCELERATION){
        //Not enough time to increase velocity
        cout << "Not enough time to increase velocity"<<endl;
        return false;
    }
    
    if (Vfinal.len() > rules.ROBOT_MAX_GROUND_SPEED){
        cout<< "Required recalc period with Vmax"<<endl;
        
        tvmaxk = 1;
        for (; tvmaxk < max_attempts; tvmaxk++){
            double tvmax = tvmaxk / rules.TICKS_PER_SECOND;
            
            acceleration = (initial_delta - rr1.velocity * (t + jp.fullTimeSec + tvmax)) * 2 / ((2* jp.fullTimeSec + t + tvmax)*( t - tvmax));
            Vfinal = rr1.velocity + acceleration * ( t - tvmax );
            if ( acceleration.len() > rules.ROBOT_ACCELERATION ){
                cout<< " Acceleration " << acceleration.len() << " is too big"<<endl;
                return false;
            }
            if (Vfinal.len() > rules.ROBOT_MAX_GROUND_SPEED)
                continue;
            path = rr1.velocity * t + acceleration * ( t - tvmax ) * ( t-tvmax ) / 2.0 + Vfinal * (jp.fullTimeSec + tvmax);
            break;
        }
        
        if (Vfinal.len() >  rules.ROBOT_MAX_GROUND_SPEED){
            cout<< "Can not achieve by veocity"<<endl;
            return false;
        }
        
        cout << "New acceleration = " << acceleration.toString() << endl;
        cout << "TVmax = " << tvmaxk << endl;
        cout << "Path len = " << path.len() << endl;
        cout << "V final " << Vfinal.toString() <<endl;
    }
    
    for ( int i = 0; i < max_attempts ; i++){
        
        if ( i < max_attempts - tvmaxk ){
            Vec3 veloc = rr1.velocity + acceleration * delta_time * rules.MICROTICKS_PER_TICK;
            rr1.action.target_velocity_x = veloc.getX();
            rr1.action.target_velocity_y = veloc.getY();
            rr1.action.target_velocity_z = veloc.getZ();
            rr1.action.jump_speed = 0;
            rr1.action.use_nitro = false;
            cout << " Velocity on acceleration going required " << veloc.toString() << endl;
            cout << " Position on start acceleration moving " << rr1.position.toString() << endl;
            cout << " Velocity on start acceleration moving " << rr1.velocity.toString() << endl;
            route.push_back(rr1);
            for (int j = 0; j < rules.MICROTICKS_PER_TICK; j++) {
                engine->moveRobot(rr1, delta_time);
                Vec3 collision_normal = engine->collide_with_arena(rr1);
                if (collision_normal == Vec3::None) {
                    rr1.touch = false;
                } else {
                    rr1.touch = true;
                    rr1.touch_normal = collision_normal;
                }
                //                SimulationEntity ball = node... ; // copy required
                //                Vec3 ballVelocity = ball.velocity;
                //                if (sim.engine->collide_entities(rr1, ball)){
                //                    if (ballVelocity.getZ() < ball.velocity())
                //                        return false; //bad collision
                //                }
            }
        } else {
            rr1.action.target_velocity_x = Vfinal.getX();
            rr1.action.target_velocity_y = Vfinal.getY();
            rr1.action.target_velocity_z = Vfinal.getZ();
            cout << " Velocity on full speed " << Vfinal.toString() << endl;
            cout << " Position on full speed moving " << rr1.position.toString() << endl;
            cout << " Velocity on full speed moving " << rr1.velocity.toString() << endl;
            rr1.action.jump_speed = 0;
            rr1.action.use_nitro = false;
            route.push_back(rr1);
            for (int j = 0; j < rules.MICROTICKS_PER_TICK; j++) {
                engine->moveRobot(rr1, delta_time);
                Vec3 collision_normal = engine->collide_with_arena(rr1);
                if (collision_normal == Vec3::None) {
                    rr1.touch = false;
                } else {
                    rr1.touch = true;
                    rr1.touch_normal = collision_normal;
                }
            }
        }
    }
    if (jp.jump_ticks > 0){
        Vec3 dp = bptarget - rr1.position;
        dp.setY(0);
        dp.mulAndApply(1/ (delta_time * rules.MICROTICKS_PER_TICK * jp.jump_ticks));
        cout << " Jump supergoodvelocity " << dp.toString() << endl;
        
        cout<< "Jump required: vel " << jp.initialVelocityY << " and ticks " << jp.jump_ticks << " and micros " << jp.jump_micros <<endl;
        rr1.action.target_velocity_x = dp.getX();
        rr1.action.target_velocity_y = dp.getY();
        rr1.action.target_velocity_z = dp.getZ();
        cout << "On jump current velocity " << rr1.velocity.toString() << endl;
        cout << "On jump current position " << rr1.position.toString() << endl;
        rr1.action.jump_speed = jp.initialVelocityY;
        rr1.action.use_nitro = false;
        route.push_back(rr1);
        for (int i = 0 ; i < jp.jump_ticks ; i++){
            for (int j = 0; j < rules.MICROTICKS_PER_TICK; j++) {
                engine->moveRobot(rr1, delta_time);
                Vec3 collision_normal = engine->collide_with_arena(rr1);
                if (collision_normal == Vec3::None) {
                    rr1.touch = false;
                } else {
                    rr1.touch = true;
                    rr1.touch_normal = collision_normal;
                }
            }
        }
    }

    cout << " Final position " << rr1.position.toString() << endl;
    cout << " Final distance " << (rr1.position - bptarget).toString() << endl;
     return true;
    
}

Vec3 Simulation::getHitPosition(SimulationEntity ball, SimulationEntity robot) {
    //attacker goes to ball position

    if ( ball.position.getZ() > ENEMY_GOAL_INACCESS_Z && abs(ball.position.getX()) > ENEMY_GOAL_INACCESS_X){
        //Enemy corner shadow zone
        cout << "Enemy corner shadow"<<endl;
        return Vec3::None;
    }
    
    double dist_k = sqrt((rules.BALL_RADIUS + rules.ROBOT_RADIUS) * (rules.BALL_RADIUS + rules.ROBOT_RADIUS) -
                         (rules.BALL_RADIUS - rules.ROBOT_RADIUS) * (rules.BALL_RADIUS - rules.ROBOT_RADIUS));
    cout << "Dist_K "<<dist_k<<endl;
    if (ball.position.getZ()< 0 && abs(ball.position.getZ()) > abs(GOAL_THRESHOLD)){
        cout << "Ball goes to my goal "<<endl;
        Vec3 myGoalDirection = robot.position - ball.position;
        myGoalDirection.setY(0);

        Vec3 hitPosition = ball.position + (myGoalDirection.normalized() * dist_k);
        hitPosition.setY(ball.position.getY() - rules.ROBOT_RADIUS);
        
        return hitPosition;
    }
    
    /**  **/
    
    Vec3 goalDirection = ENEMY_GOAL_TARGET - ball.position;
    double lenBG2 = goalDirection.lenPowered2();
    double lenRB2 = (ball.position - robot.position).lenPowered2();
    double lenRG2 = (ENEMY_GOAL_TARGET - robot.position).lenPowered2();
    double lenRBBG2 = (sqrt(lenRB2) + sqrt(lenBG2))*(sqrt(lenRB2) + sqrt(lenBG2));
    
    if (abs(lenRG2 - (lenRB2 + lenBG2)) < abs(lenRG2 - lenRBBG2) && abs(goalDirection.getZ()) < ENEMY_GOAL_INACCESS_Z){
        // looks like angle too big
        // attack to mirrored position
        if (ball.position.getX() < robot.position.getX()){
            
            ENEMY_GOAL_TARGET.setX(rules.arena.width);
            cout << "Update ENEMY_GOAL_TARGET 1 "<<ENEMY_GOAL_TARGET.toString()<<endl;
        } else {
            ENEMY_GOAL_TARGET.setX(-rules.arena.width);
            cout << "Update ENEMY_GOAL_TARGET 2 "<<ENEMY_GOAL_TARGET.toString()<<endl;
        }
        goalDirection = ENEMY_GOAL_TARGET - ball.position - ball.velocity;
    }
    cout << "Goal direction "<<goalDirection.toString()<<endl;
    //attack goal directly
    goalDirection.setY(0);
    
    Vec3 hitPosition = ball.position - (goalDirection.normalized() * dist_k);
    cout << "Initial hit position "<<hitPosition.toString()<<endl;
    hitPosition.setY(ball.position.getY() - rules.ROBOT_RADIUS);
    cout << "Hit position updated "<<hitPosition.toString()<<endl;
    return hitPosition;
}

void Simulation::moveRobotAndAdjustNextNode(SimulationEntity &robot, TreeNode *childNode) {
    if (robot.velocity.len() > 0) {
        for (int j = 0; j < rules.MICROTICKS_PER_TICK; j++) {
            engine->moveRobot(robot, delta_time);

            Vec3 collision_normal = engine->collide_with_arena(robot);
            if (collision_normal == Vec3::None) {
                robot.touch = false;
            } else {
                robot.touch = true;
                robot.touch_normal = collision_normal;
            }
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
//Target point is robot center position in moment of hit
JumpParams Simulation::getJumpParams(Vec3 targetPoint, bool useMaxVelocity) const{
    
    double hitPositionY = targetPoint.getY() - rules.ROBOT_RADIUS;
//    cout<< "Target height "<< hitPositionY<<endl;
    if (hitPositionY <= 0){
        return JumpParams(0, 0, 0);
    }
    
    double Vo = 0;
    double h = 0;
    double t = 0;
    if (useMaxVelocity) { //Using max speed just for check achievement
        Vo = rules.ROBOT_MAX_JUMP_SPEED;
        while ( t < 31 && h <= hitPositionY){
            ++t;
            //        cout << " t " <<t<<endl;
            double time = t / rules.TICKS_PER_SECOND;
            //        cout << " time = "<<time<<endl;
            h = Vo * time - rules.GRAVITY * time * time / 2.0;
            //        cout << " jump height "<< h << endl;
            if ( t < 31 && h >= hitPositionY ){
                
                double h1 = Vo * ( t - 1 ) / rules.TICKS_PER_SECOND - rules.GRAVITY * ( t - 1 )*( t - 1 )/ (rules.TICKS_PER_SECOND * rules.TICKS_PER_SECOND * 2);
                //            cout << "h1 "<< h1<< endl;
                for ( int i = 1; i <= rules.MICROTICKS_PER_TICK; i++){
                    h1 += Vo * delta_time - rules.GRAVITY * delta_time * delta_time /2.0;
                    //                cout << "h1 "<<h1<<endl;
                    if (h1 > hitPositionY){
                        //                    cout<< " Full time = " << full_time << " where "<< t-1 << " ticks and "<< i << " microticks"<<endl;
                        //                    cout << " Real initial jump speed "<< V1 << endl;
                        return JumpParams(Vo, t - 1, i);
                    }
                }
            }
        }
    }
    
    
    Vo = 0;
    h = 0;
    t = 0;
    while ( t < 31 && Vo <= rules.ROBOT_MAX_JUMP_SPEED && h <= hitPositionY){
        ++t;
        //        cout << " t " <<t<<endl;
        double time = t / rules.TICKS_PER_SECOND;
        //        cout << " time = "<<time<<endl;
       
        Vo = rules.GRAVITY * time; // try to hit with zero Y velocity
       
        h = Vo * time - rules.GRAVITY * time * time / 2.0;
        //        cout << " jump height "<< h << endl;
        if ( t < 31 && Vo <= rules.ROBOT_MAX_JUMP_SPEED && h >= hitPositionY ){
            //Velocity too big and robot becomes higher than required
            
            double V1 = hitPositionY/time + rules.GRAVITY * time / 2;
            return JumpParams(V1, t, 0);
            
        }
    }
    return JumpParams();
}

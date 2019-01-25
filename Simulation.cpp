
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
    
    ENEMY_GOAL_SHADOW_CORNER = ENEMY_GOAL_INACCESS_X + rules.arena.depth / 2.0 - rules.BALL_RADIUS;
    dist_k = sqrt((rules.BALL_RADIUS + rules.ROBOT_RADIUS) * (rules.BALL_RADIUS + rules.ROBOT_RADIUS) -
                         (rules.BALL_RADIUS - rules.ROBOT_RADIUS) * (rules.BALL_RADIUS - rules.ROBOT_RADIUS));
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
        cout << "SetTick " << current_tick << endl;
        if (baseNode->children.size() > 0) {
//            cout << "Call known nodes" << endl;
            shared_ptr<TreeNode> &tn = baseNode->children[baseNode->children.size() - 1];

            if (tn->state.current_tick == current_tick &&
                tn->state.ball.position == Vec3(g.ball.x, g.ball.y, g.ball.z)) {
                bool action_required = false;
                for (const SimulationEntity &se : tn->state.robots) {
                    if (se.teammate && !se.action_set) {
                        action_required = true;
                        cout << " Action required " << action_required << endl;
                    }
                    //DEBUG comment on publish
                    if (se.teammate){
                        for (const Robot &r: g.robots ){
                            if (r.id == se.id && (!isEqual(se.position.getX(),r.x) || !isEqual(se.position.getY(),r.y) || !isEqual(se.position.getZ(),r.z  ))){
                                cout << " ROBOT position WRONG!!! expect " << se.position.toString() <<  " REAL " << r.x <<":"<<r.y<<":"<<r.z <<endl;
                            }
                        }
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

        cout << "Simulation wrong: " << current_tick << endl;

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
    int skipForNowId = -1;
    int freeGoing = 0;

    bool isBallFlyInitially = tn->state.ball.position.getY() > rules.ROBOT_RADIUS*2 + rules.BALL_RADIUS;
    Vec3 achievementFlyingBallPosition = Vec3::None;
    int achievementFlyingBallTick = -1;


    while (tn->children.size() > 0) {
        route.clear();
        for (SimulationEntity &se: tn->state.robots) {
            if (!se.teammate)
                continue;

            if (se.position.getZ() > tn->state.ball.position.getZ() && se.id!= goalKeeperId){
                skipForNowId = se.id;
            }
            
            if (se.touch && se.id != skipForNowId &&
                ((tn->state.ball.position.getZ() < 0 && se.id == goalKeeperId) ||
                 se.id != goalKeeperId)) {
        
                    Vec3 ballHitPosition = getHitPosition(tn->state.ball, se);
                    
                    cout << "Hit position " << ballHitPosition.toString() << " for tick " << tn->state.current_tick << endl;
                    
                    if (ballHitPosition == Vec3::None){
                        continue;
                    }
                    
                    Vec3 rp = se.position;
                    SimulationEntity ball = tn->state.ball;
  
                    
                    hitAchieved = checkAchievement(se, ballHitPosition,
                                                                tn->state.current_tick - current_tick, route);
                    
  
                    if (hitAchieved) {
                        cout << " For Attacker " << se.id << endl;
                        attackerId = se.id;
                        break;
                    } else {
                        if ( ball.position.getY()< rules.ROBOT_RADIUS*1.75 + rules.BALL_RADIUS){

                            double XZ =sqrt((rules.ROBOT_RADIUS + rules.BALL_RADIUS)*(rules.ROBOT_RADIUS + rules.BALL_RADIUS)- (ball.position.getY()- rules.ROBOT_RADIUS)*(ball.position.getY()- rules.ROBOT_RADIUS));

                            hitAchieved = checkAchievement(se, Vec3(ball.position.getX() , 1, ball.position.getZ()-XZ),
                                                                                           tn->state.current_tick - current_tick, route);
                            if (hitAchieved) {
                                cout << " For Attacker ground " << se.id << endl;
                                attackerId = se.id;
                                break;
                            }
                        }
                    }

                }
            if ( tn->state.ball.position.getY() < rules.ROBOT_RADIUS*2 + rules.BALL_RADIUS && achievementFlyingBallTick){
                achievementFlyingBallPosition = tn->state.ball.position;
                achievementFlyingBallTick = tn->state.current_tick;
            }
            if (!se.touch){
                se.action.jump_speed = 0.0;
                se.action.use_nitro = false;
                
                cout << "Target velocity for flyer " << se.velocity.toString() << " and id " << se.id << endl;
                cout << "Current position for flyer " << se.position.toString() << " and node " << tn->state.current_tick << endl;
                se.action.target_velocity_x = se.velocity.getX();
                se.action.target_velocity_y = se.velocity.getY();
                se.action.target_velocity_z = se.velocity.getZ();
                se.action_set = true;
                freeGoing++;
                moveRobotAndAdjustNextNode(se, tn->children[0].get());
                cout << endl;
            }
        }
        if (attackerId != -1) {
            break;
        }

        tn = tn->children[0].get();
    }

    if (attackerId == goalKeeperId) {
        for (const SimulationEntity &se: baseNode->state.robots) {
            if (se.teammate && se.id != attackerId) {
                goalKeeperId = se.id;
                break;
            }
        }
    }
    Vec3 lastKnownBallPosition = Vec3::None;
    lastKnownBallPosition = tn->state.ball.position;
    tn = baseNode.get();

    cout << "ACTIONS " << tn->state.current_tick << endl;
    cout << " tn->state.current_tick "<< tn->state.current_tick << endl;
    
    if (hitAchieved) {
        cout<< " +++++++ TICK COLLISION EXISTS +++++++++"<<endl;
        int i = 0;
        while (tn->children.size() > 0 && i < route.size()) {
            TreeNode *childNode = tn->children[0].get();
            cout << "Action for tick " << tn->state.current_tick <<endl;
            for (SimulationEntity &robot: tn->state.robots) {
                if (robot.id == attackerId) {
                    //COPY FROM ROUTE
                    
                    robot.action = route.at(i).action;
                    robot.position = route.at(i).position;
                    robot.velocity = route.at(i).velocity;
                    robot.action_set = true;
                    i++;
                    cout << "Attacker id "<< robot.id << endl;
                    cout << "Robot action target velocity "<< robot.action.target_velocity_x << ";"
                    << robot.action.target_velocity_y << ";" << robot.action.target_velocity_z << endl;
                    cout << "Robot action target jumpspeed "<< robot.action.jump_speed << endl;
                    
                    
                } else if (robot.id == goalKeeperId) {

                    //TODO: collision with attacker

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
                    if (tn->state.ball.position.getY() <= rules.ROBOT_RADIUS * 2 + rules.BALL_RADIUS){
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
                    }
                    robot.action.jump_speed = 0.0;
                    robot.action.use_nitro = false;
                    Vec3 del = (target - robot.position).normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    if (del.len() > 1) {
                        del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                    }
                    cout << "Target velocity " << del.toString() << endl;
                    robot.action.target_velocity_x = del.getX();
                    robot.action.target_velocity_y = del.getY();
                    robot.action.target_velocity_z = del.getZ();
                    robot.action_set = true;
                    moveRobotAndAdjustNextNode(robot, childNode);
                }
            }
            cout<<endl;
            tn = childNode;
        }
        cout<< " ------- TICK COLLISION EXISTS END -------"<<endl;
        
    } else {
        cout << " +++++++++ FREE GOING +++++++++++"<<endl;
//        TreeNode *targetBallNode = tn;

        bool allTimeFly = true;

        if (freeGoing == 0)
            freeGoing = 20;

        for (int j = 0; j < freeGoing; j++) {
//            targetBallNode = targetBallNode->children[tn->children.size() - 1].get();
            if ( tn->state.ball.position.getY() <= rules.ROBOT_RADIUS * 2 + rules.BALL_RADIUS){
                allTimeFly = false;
            }
        }
        for (int i = 0; i < freeGoing; i++) {
            TreeNode *childNode = tn->children[tn->children.size() - 1].get();
            cout << "Action for tick " << tn->state.current_tick <<endl;
            for (SimulationEntity &robot: tn->state.robots) {
                Vec3 target = defaultGoalKeeperPosition; // FIND BALL x GOAL position
                
                if (robot.id == goalKeeperId) {
                    cout << " Goal keeper id " << goalKeeperId <<endl;
                    if (tn->state.ball.velocity.getZ() < 0){
                        double k = tn->state.ball.velocity.getZ() / tn->state.ball.velocity.getX();
                        double b = tn->state.ball.position.getZ() - k * tn->state.ball.position.getX();
                        double x = (-rules.arena.depth/2 - b ) / k;
                        if (abs(x) <= abs(rules.arena.goal_width /2)){
                            target.setX(x);
                        }
                    }
                    if (!allTimeFly){
                        if (tn->state.ball.position.getZ() < robot.position.getZ() + rules.BALL_RADIUS ){
                            if (abs(target.getX() - robot.position.getX()) < radsum) {
                                target = robot.position;
//                                if (abs(robot.position.getX()) < abs(targetBallNode->state.ball.position.getX()) + radsum) {
//                                    if (robot.position.getX() < targetBallNode->state.ball.position.getX())
                                if (abs(robot.position.getX()) < abs(lastKnownBallPosition.getX()) + radsum) {
                                    if (robot.position.getX() < lastKnownBallPosition.getX())
                                        target.setX(target.getX() - radsum * 2);
                                    else
                                        target.setX(target.getX() + radsum * 2);
                                } else {
                                    target.setX(0.0);
                                }
                            }
                        }
                    }
                } else {
                    cout << " Last known attacker id " << robot.id << endl;
//                    target = targetBallNode->state.ball.position;
                    target = lastKnownBallPosition;
                    cout << " TArgt ball node position "<< target.toString()<< endl;
                    cout<< " Robot current speed " << robot.velocity.toString() << endl;
                    cout << " ball current pos "<< tn->state.ball.position.toString() << endl;
                    target.setY(1.0);
                    
                    cout << " Default--- Go 2 ball LAST ATTACKER " << target.toString() << " parentNode tick: "
                    << tn->state.current_tick << " robot id "<< robot.id
                    << endl;
                    
                    //TODO: check for ball Y
                    if (!allTimeFly){
                        target.setZ(target.getZ() - rules.BALL_RADIUS * 2);
                        if (tn->state.ball.position.getZ() < robot.position.getZ() + 1 || target.getZ() < robot.position.getZ() + 1){
                            if (abs(target.getX() - robot.position.getX()) < radsum) {
                                target = robot.position;
//                                if (abs(robot.position.getX()) < abs(targetBallNode->state.ball.position.getX()) + radsum) {
//                                    if (robot.position.getX() < targetBallNode->state.ball.position.getX())
                                if (abs(robot.position.getX()) < abs(lastKnownBallPosition.getX()) + radsum) {
                                    if (robot.position.getX() < lastKnownBallPosition.getX())
                                        target.setX(target.getX() - radsum * 2);
                                    else
                                        target.setX(target.getX() + radsum * 2);
                                } else {
                                    target.setX(0.0);
                                }
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
                if (del.len() > 1 && i < 10) {
                    del = del.normalized() * rules.ROBOT_MAX_GROUND_SPEED;
                }
                robot.action.target_velocity_x = del.getX();
                robot.action.target_velocity_y = del.getY();
                robot.action.target_velocity_z = del.getZ();
                robot.action_set = true;
                moveRobotAndAdjustNextNode(robot, childNode);
            }
            tn = childNode;
            cout<<endl;
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

bool Simulation::checkAchievement(SimulationEntity rr1, Vec3 bptarget, int max_attempts, vector<SimulationEntity>& route) {
    cout << endl << endl;
    
    cout << "## Check achievement for robot on position "<< rr1.position.toString() << endl
    << "velocity " << rr1.velocity.toString() << endl
    << "and bp target "<< bptarget.toString() << endl;
    cout << "Available delta time " << max_attempts << endl;
    
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
//    Vec3 tv = initial_delta.normalized();
//    tv.mulAndApply(rules.ROBOT_MAX_GROUND_SPEED);
    JumpParams jp = getJumpParams(bptarget, false || bptarget.getZ()< -rules.arena.depth / 4.0 || bptarget.getY() > rules.ROBOT_RADIUS*2);
    if (jp.jump_ticks == -1){
        jp = getJumpParams(bptarget, true);
        if (jp.jump_ticks == -1){
            cout << "can not jump so high"<<endl;
            return false;
        }
    }
    
    if (max_attempts - jp.jump_ticks < 0){
        // not enough time for jump
        if (jp.initialVelocityY != rules.ROBOT_MAX_JUMP_SPEED)
            jp = getJumpParams(bptarget, true);
        if (max_attempts - jp.jump_ticks < 0){
            cout << "Not enough time for jump"<<endl;
            return false;
            
        }
    }
    
    max_attempts -= jp.jump_ticks;
    
    cout << "Jump ticks "<< jp.jump_ticks<<endl;
    cout << "Full jump time in sec "<< jp.fullTimeSec <<endl;
    
    double attempts_t = ((double)max_attempts) / rules.TICKS_PER_SECOND;
    
    //TODO: review rightPart!
    double rightPartTime = ((initial_delta - rr1.velocity * (jp.fullTimeSec + attempts_t + delta_time)) / rules.ROBOT_ACCELERATION).len();
    
    if (rightPartTime > attempts_t * ( jp.fullTimeSec + attempts_t / 2)){
        
        if (jp.initialVelocityY != rules.ROBOT_MAX_JUMP_SPEED)
            jp = getJumpParams(bptarget, true);
        if (rightPartTime > attempts_t * ( jp.fullTimeSec + attempts_t / 2)){
        
            cout << "not enough time for velocity change"<<endl;
            return false;
            
        }
    }
    double accelTime = attempts_t / 2;
    double deltaTime = accelTime;
    double fullAccelerationTime = 0;
    bool less = true;
    do{
        fullAccelerationTime = accelTime * (jp.fullTimeSec - accelTime / 2 + attempts_t);
        if ( fullAccelerationTime > rightPartTime ){
            if ( !less ){
                less = true;
                deltaTime /= 2;
            }
                
            accelTime -= deltaTime;
        } else if ( fullAccelerationTime < rightPartTime ){
            if (less){
                less = false;
                deltaTime /= 2;
            }
            accelTime += deltaTime;
        }
        cout << "New acceleration time " << accelTime << endl;
    } while ( rightPartTime != fullAccelerationTime && deltaTime > delta_time );
    
    Vec3 Vfinal = rr1.velocity + initial_delta.normalized() * rules.ROBOT_ACCELERATION * accelTime;
    cout << "Velocity final "<< Vfinal.toString()<< endl;
    if (Vfinal.len()> rules.ROBOT_MAX_GROUND_SPEED){
        cout << "Too big velocity "<<endl;
        return false;
    }
    
        cout << "Position after 1 ticks " << (rr1.position + rr1.velocity / rules.TICKS_PER_SECOND + initial_delta.normalized() * rules.ROBOT_ACCELERATION / (rules.TICKS_PER_SECOND * rules.TICKS_PER_SECOND * 2)).toString() << endl;

        cout << "Position after 5 ticks " << (rr1.position + rr1.velocity * 5 / rules.TICKS_PER_SECOND + initial_delta.normalized() * rules.ROBOT_ACCELERATION * 25 /(rules.TICKS_PER_SECOND * rules.TICKS_PER_SECOND * 2)).toString() << endl;


    for ( int i = 0; i < max_attempts ; i++){
        
        if (jp.jump_ticks >0 && i == max_attempts -1)
            break;
        Vec3 direction = (bptarget_plain - rr1.position).normalized();
        Vec3 vel = direction * Vfinal.len();
        
        rr1.action.target_velocity_x = vel.getX();
        rr1.action.target_velocity_y = vel.getY();
        rr1.action.target_velocity_z = vel.getZ();
        rr1.action.jump_speed = 0;
        rr1.action.use_nitro = false;
        cout << "++"<<endl<<" Velocity on acceleration going required " << Vfinal.toString() << endl;
        cout << " Position on start acceleration moving " << rr1.position.toString() << endl;
        cout << " Velocity on start acceleration moving " << rr1.velocity.toString() << endl <<"--"<<endl;
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
    if (jp.jump_ticks > 0){
        Vec3 dp = Vfinal;
        if (max_attempts != 0){
            dp = bptarget - rr1.position;
            dp.setY(0);
            dp.mulAndApply(1/ (delta_time * rules.MICROTICKS_PER_TICK * (jp.jump_ticks + 1)));
            cout << " Jump supergoodvelocity " << dp.toString() << endl;
            cout << " Start one plain tick "<<endl;
            rr1.action.target_velocity_x = dp.getX();
            rr1.action.target_velocity_y = dp.getY();
            rr1.action.target_velocity_z = dp.getZ();
            cout << "On jump current velocity " << rr1.velocity.toString() << endl;
            cout << "On jump current position " << rr1.position.toString() << endl;
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
        
        dp = bptarget - rr1.position;
        dp.setY(0);
        dp.mulAndApply(1/ (delta_time * rules.MICROTICKS_PER_TICK * jp.jump_ticks));
        cout << " Jump supergoodvelocity2 " << dp.toString() << endl;
        cout << " Start one plain tick "<<endl;
        cout<< "Jump required: vel " << jp.initialVelocityY << " and ticks " << jp.jump_ticks << " and micros " << jp.jump_micros <<endl;
        rr1.action.target_velocity_x = dp.getX();
        rr1.action.target_velocity_y = dp.getY();
        rr1.action.target_velocity_z = dp.getZ();
        cout << "On jump current velocity " << rr1.velocity.toString() << endl;
        cout << "On jump current position " << rr1.position.toString() << endl;
        rr1.action.jump_speed = jp.initialVelocityY;
        rr1.action.use_nitro = false;
        
        for (int i = 0 ; i < jp.jump_ticks ; i++){
            if (i != 0){
                rr1.action.jump_speed = 0;
            }
            if (i == jp.jump_ticks-1){
                if (rr1.position.getZ()>= abs(GOAL_THRESHOLD) )
                    rr1.action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED/2;
                else
                    rr1.action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
            }
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

    cout << " Final position " << rr1.position.toString() << endl;
    cout << " Final distance " << (bptarget - rr1.position).toString() << endl;
     return true;

}

Vec3 Simulation::getHitPosition(SimulationEntity ball, SimulationEntity robot) {
    //attacker goes to ball position

//    if ( ball.position.getZ() + abs(ball.position.getX()) > ENEMY_GOAL_SHADOW_CORNER){
//        //Enemy corner shadow zone
//        cout << "Enemy corner shadow"<<endl;
//        return Vec3::None;
//    }
//    
    
    cout << "Dist_K "<<dist_k<<endl;
    Vec3 ballVelNormilized = ball.velocity.normalized();
    cout << "Ball velocity "<< ballVelNormilized.toString()<< endl;
    cout << "Ball position "<< ball.position.toString()<< endl;
   
    

//    if (ball.velocity.getZ() < 0 ){
//        if ( ball.position.getY()> rules.ROBOT_RADIUS*1.75 + rules.BALL_RADIUS){
//            double radsum = rules.ROBOT_RADIUS + rules.BALL_RADIUS;
//            double dY = ball.position.getY() - rules.ROBOT_RADIUS;
//            double XZ = sqrt( radsum * radsum - dY * dY );
//
//            return Vec3(ball.position.getX() , 1, ball.position.getZ() - XZ );
//        }
//    }

    
    
    if (ball.velocity.getZ() < 0 && abs(ball.position.getZ()) > abs(GOAL_THRESHOLD) && ball.position.getZ() <= 0){
        cout << "Ball goes to my goal "<<endl;
        Vec3 myGoalDirection = robot.position - ball.position;
        cout << "My Goal direction "<< myGoalDirection.toString() << endl;
        myGoalDirection.normAndApply();
        cout << "My Goal direction normilized "<< myGoalDirection.toString() << endl;
        ballVelNormilized.setY(0);
        ballVelNormilized.addAndApply(myGoalDirection);
        ballVelNormilized.normAndApply();
        ballVelNormilized.mulAndApply(dist_k);
        

        Vec3 hitPosition = ball.position + ballVelNormilized;
        
        return hitPosition;
    }
    
    /** TODO: normalize vectors before angle compare + velocity check **/
    
    Vec3 goalDirection = ENEMY_GOAL_TARGET - ball.position;
//    double lenBG2 = goalDirection.lenPowered2();
//    double lenRB2 = (ball.position - robot.position).lenPowered2();
//    double lenRG2 = (ENEMY_GOAL_TARGET - robot.position).lenPowered2();
//    double lenRBBG2 = (sqrt(lenRB2) + sqrt(lenBG2))*(sqrt(lenRB2) + sqrt(lenBG2));
//
//    if (abs(lenRG2 - (lenRB2 + lenBG2)) < abs(lenRG2 - lenRBBG2) && abs(goalDirection.getZ()) < ENEMY_GOAL_INACCESS_Z){
//        // looks like angle too big
//        // attack to mirrored position
//        if (ball.position.getX() < robot.position.getX()){
//
//            ENEMY_GOAL_TARGET.setX(rules.arena.width);
//            cout << "Update ENEMY_GOAL_TARGET 1 "<<ENEMY_GOAL_TARGET.toString()<<endl;
//        } else {
//            ENEMY_GOAL_TARGET.setX(-rules.arena.width);
//            cout << "Update ENEMY_GOAL_TARGET 2 "<<ENEMY_GOAL_TARGET.toString()<<endl;
//        }
//        goalDirection = ENEMY_GOAL_TARGET - ball.position;
//    }
    cout << "Goal direction "<<goalDirection.toString()<<endl;
    //attack goal directly
    goalDirection.setY(0);
    
    goalDirection.normAndApply();
    ballVelNormilized.setY(0);
    cout << "Goal direction "<<goalDirection.toString()<<endl;
    goalDirection.subAndApply(ballVelNormilized);
    cout << "Goal direction "<<goalDirection.toString()<<endl;
    goalDirection.normAndApply();
    cout << "Goal direction "<<goalDirection.toString()<<endl;
    goalDirection.mulAndApply(2.9);
    cout << "Goal direction "<<goalDirection.toString()<<endl;
    
    Vec3 hitPosition = ball.position - goalDirection;
    if (abs(hitPosition.getX()) > rules.arena.width/2 - rules.ROBOT_RADIUS){
        cout << "Too big X" << endl;
        return Vec3::None;
    }
    
        Vec3 nd = (ball.position - robot.position).normalized();
        nd.mulAndApply(3);
        Vec3 nnd = Vec3(nd.getX(), 0, nd.getY());
        goalDirection.setY(0);
        goalDirection.normAndApply();
        goalDirection.mulAndApply(nnd.len());
        goalDirection.addAndApply(Vec3(0, nd.getY(), 0));
        hitPosition = ball.position - goalDirection;
        cout << "hit position' "<<hitPosition.toString()<<endl;
    
    
    cout << "Initial hit position "<<hitPosition.toString()<<endl;
//    hitPosition.setY(ball.position.getY() - rules.ROBOT_RADIUS);
//    cout << "Hit position updated "<<hitPosition.toString()<<endl;
    return hitPosition;
}

void Simulation::moveRobotAndAdjustNextNode(SimulationEntity robot, TreeNode *childNode) {
    if (robot.action_set) {
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
//                    cout << " t " <<t<<endl;
            double time = t / rules.TICKS_PER_SECOND;
//                    cout << " time = "<<time<<endl;
            h =  (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * Vo /rules.ROBOT_MAX_JUMP_SPEED + Vo * time - rules.GRAVITY * time * time / 2.0;
//                    cout << " jump height "<< h << endl;
            if ( t < 31 && h >= hitPositionY ){
                
                double h1 = (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * Vo /rules.ROBOT_MAX_JUMP_SPEED  + Vo * ( t - 1 ) / rules.TICKS_PER_SECOND - rules.GRAVITY * ( t - 1 )*( t - 1 )/ (rules.TICKS_PER_SECOND * rules.TICKS_PER_SECOND * 2);
//                            cout << "h1 "<< h1<< endl;
                for ( int i = 1; i <= rules.MICROTICKS_PER_TICK; i++){
                    h1 += Vo * delta_time - rules.GRAVITY * delta_time * delta_time /2.0;
//                                    cout << "h1 "<<h1<<endl;
                    if (h1 > hitPositionY){

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
//                cout << " t " <<t<<endl;
        double time = t / rules.TICKS_PER_SECOND;
//                cout << " time = "<<time<<endl;
       
        Vo = rules.GRAVITY * time; // try to hit with zero Y velocity
       
        h = (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * (Vo/(1.0 + rules.ROBOT_ARENA_E)) /rules.ROBOT_MAX_JUMP_SPEED  + Vo * (time - delta_time) - rules.GRAVITY * time * time / 2.0;
//                cout << " jump height "<< h << endl;
        if ( t < 31 && Vo <= rules.ROBOT_MAX_JUMP_SPEED && h >= hitPositionY ){
            //Velocity too big and robot becomes higher than required
            
            //double V1 = (hitPositionY )/time + rules.GRAVITY * time / 2;
            double V1 = (hitPositionY + rules.GRAVITY * time*time/2.0)/ ((rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS)/((1.0+rules.ROBOT_ARENA_E)*rules.ROBOT_MAX_JUMP_SPEED)+(time-delta_time));
            double Vinitial = V1/(1.0+rules.ROBOT_ARENA_E);
//            cout << "V1 " << V1<< endl;
//            cout<< "Vinitial "<< Vinitial << endl;
            return JumpParams(Vinitial, t, 0);
            
        }
    }
    return JumpParams();
}

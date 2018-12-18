#include "MyStrategy.h"

#include "CVL_Utils.h"
#include <iostream>
#include <sstream>

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
    
    
    dumpBall(game.ball);
    dumpRobot(me);
    std::stringstream log;
    
    writeLog(log);
}


void MyStrategy::dumpRobot(const model::Robot& r){
    std::stringstream ss;
    ss << " ROBOT: id: *" << r.id <<"*";
    ss << " player_id: *" << r.player_id << "*";
    ss << " is_teammate: *" <<r.is_teammate << "*";
    ss << " coord: ("<<r.x<<";"<<r.y<<":"<<r.z<<")";
    ss << " velocity: ("<<r.velocity_x<<";"<<r.velocity_y<<";"<<r.velocity_z<<")";
    ss << " radius:" <<r.radius;
    ss << " nitro:" <<r.nitro_amount;
    ss << " touch:" <<r.touch;
    ss << " touch_normal: x:"<<r.touch_normal_x<<"Y:"<<r.touch_normal_y<<"z:"<<r.touch_normal_z<<std::endl;
    writeLog(ss);
}

void MyStrategy::dumpBall(const model::Ball& b){
    std::stringstream ss;
    ss << " BALL radius:"<<b.radius;
    ss << " coord:("<<b.x<<";"<<b.y<<";"<<b.z<<")";
    ss << " velocity:("<<b.velocity_x<<";"<<b.velocity_y<<";"<<b.velocity_z<<")"<<std::endl;
    writeLog(ss);
}

#include <memory>

#include "Runner.h"
#include "MyStrategy.h"
#include <iomanip>

using namespace model;
using namespace std;

void checkAchievement(SimulationEntity &rr1, Vec3 bptarget, Rules *rules, int max_attempts) {
    double delta_time = 1.0 / 6000.0;

    cout << "Bparget " << bptarget.toString() << endl;
    cout << delta_time << endl;

    Vec3 initial_delta = bptarget - rr1.position;
    Vec3 action_target_velocity = (bptarget - rr1.position);
    action_target_velocity = action_target_velocity.normalized() * rules->ROBOT_MAX_GROUND_SPEED;

    cout << "initial delta : " << initial_delta.len() << endl;
    cout << "Action tv" << action_target_velocity.toString() << " len " << action_target_velocity.len() << endl;

    double distance_initial = initial_delta.len();
    double distance_after = initial_delta.len();
    int step = 0;
    while (distance_after <= distance_initial) {
        distance_initial = distance_after;
        for (int j = 0; j < 100; j++) {
            Vec3 target_velocity = clamp(action_target_velocity, rules->ROBOT_MAX_GROUND_SPEED);
            target_velocity = target_velocity - (rr1.touch_normal * dot(rr1.touch_normal, target_velocity));
            Vec3 target_velocity_change = target_velocity - rr1.velocity;

            if (target_velocity_change.len() > 0.0) {

                double acceleration = rules->ROBOT_ACCELERATION * max(0.0, rr1.touch_normal.getY());
                rr1.velocity = rr1.velocity + clamp(target_velocity_change.normalized() * acceleration * delta_time,
                                                    target_velocity_change.len());
            }

            rr1.velocity = clamp(rr1.velocity, rules->MAX_ENTITY_SPEED);
            rr1.position = rr1.position + rr1.velocity * delta_time;
        }
        step++;
        distance_after = (rr1.position - bptarget).len();
        cout << "vel " << rr1.velocity.toString() << " vel " << rr1.velocity.len() << endl;
        cout << "pos " << rr1.position.toString() << endl;
        cout << "delta : " << distance_after << " on tick " << step << endl;
    }

    cout << "delta : " << (rr1.position - bptarget).len() << " step: " << step << endl;
}

void blabla(Ball ball, Robot r1, Rules *rules) {
    SimulationEntity se;
    se.velocity = Vec3(ball.velocity_x, ball.velocity_y, ball.velocity_z);
    se.position = Vec3(ball.x, ball.y, ball.z);

    SimulationEntity rob;
    rob.velocity = Vec3(r1.velocity_x, r1.velocity_y, r1.velocity_z);
    rob.touch_normal = Vec3(r1.touch_normal_x, r1.touch_normal_y, r1.touch_normal_z);
    rob.position = Vec3(r1.x, r1.y, r1.z);

    double delta_time = 1.0 / 6000.0;
    int first_attempt = 50;

    Vec3 bptarget = se.position + se.velocity * first_attempt * delta_time * 100;

    checkAchievement(rob, bptarget, rules, first_attempt);
}

double jumpSpeed(double targetHeight, Rules * rules){
    double dt = 1.0 / (rules->TICKS_PER_SECOND);
    targetHeight = targetHeight - rules->ROBOT_RADIUS;
    cout<< "Target height "<< targetHeight<<endl;
    
    double targetTime = sqrt(targetHeight * 2.0/ rules->GRAVITY);
    cout << " Target time " << targetTime << endl;
    
    double jumpspeed = rules->GRAVITY * targetTime;
    cout << " Jump speed "<< jumpspeed << endl;
    
    double ticksd = targetTime/dt;
    int target_ticks = (int) ticksd;
    cout << " Ticks "<< target_ticks << endl;
    return jumpspeed;
}

void testRun1() {
    Simulation sim;
    unique_ptr<Game> game(new Game());
    unique_ptr<Rules> rules(new Rules());
    rules->arena = Arena();
    rules->arena.width = 60.0;
    rules->arena.height = 20.0;
    rules->arena.depth = 80.0;
    rules->arena.bottom_radius = 3.0;
    rules->arena.top_radius = 7.0;
    rules->arena.corner_radius = 13.0;
    rules->arena.goal_top_radius = 3.0;
    rules->arena.goal_width = 30.0;
    rules->arena.goal_height = 10.0;
    rules->arena.goal_depth = 10.0;
    rules->arena.goal_side_radius = 1.0;
    rules->max_tick_count = 18000;
    rules->team_size = 2;
    rules->ROBOT_MIN_RADIUS = 1.0;
    rules->ROBOT_MAX_RADIUS = 1.05;
    rules->ROBOT_MAX_JUMP_SPEED = 15.0;
    rules->ROBOT_ACCELERATION = 100.0;
    rules->ROBOT_NITRO_ACCELERATION = 30.0;
    rules->ROBOT_MAX_GROUND_SPEED = 30.0;
    rules->ROBOT_ARENA_E = 0.0;
    rules->ROBOT_RADIUS = 1.0;
    rules->ROBOT_MASS = 2.0;
    rules->TICKS_PER_SECOND = 60;
    rules->MICROTICKS_PER_TICK = 100;
    rules->RESET_TICKS = 120;
    rules->BALL_ARENA_E = 0.7;
    rules->BALL_RADIUS = 2.0;
    rules->BALL_MASS = 1.0;
    rules->MIN_HIT_E = 0.4;
    rules->MAX_HIT_E = 0.5;
    rules->MAX_ENTITY_SPEED = 100.0;
    rules->MAX_NITRO_AMOUNT = 100.0;
    rules->START_NITRO_AMOUNT = 50.0;
    rules->NITRO_POINT_VELOCITY_CHANGE = 0.6;
    rules->NITRO_PACK_X = 20.0;
    rules->NITRO_PACK_Y = 1.0;
    rules->NITRO_PACK_Z = 30.0;
    rules->NITRO_PACK_RADIUS = 0.5;
    rules->NITRO_PACK_AMOUNT = 100.0;
    rules->NITRO_PACK_RESPAWN_TICKS = 600;
    rules->GRAVITY = 30.0;

    game->ball = Ball();
    game->ball.x = 0.0;
    game->ball.y = 1.0;
    game->ball.z = 0.0;
    game->ball.velocity_x = 1.0;
    game->ball.velocity_y = 0.0;
    game->ball.velocity_z = 1.0;
    game->ball.radius = 2.0;

    game->current_tick = 0;
    Robot r1 = Robot();
    r1.id = 1;
    r1.player_id = 0;
    r1.x = 7.856392606191905;
    r1.y = 1.0;
    r1.z = -18.39231076339711;
    r1.velocity_x = -1.0;
    r1.velocity_y = 0.0;
    r1.velocity_z = 0.0;
    r1.radius = 1.0;
    r1.nitro_amount = 0.0;
    r1.touch_normal_x = 0.0;
    r1.touch_normal_y = 1.0;
    r1.touch_normal_z = 0.0;
    r1.touch = true;
    r1.is_teammate = true;
    game->robots.push_back(r1);

    //    Robot r2 = Robot();
    //    r2.id = 2;
    //    r2.player_id = 0;
    //    r2.x= -5.860171;
    //    r2.y =1.0;
    //    r2.z=-19.122196;
    //    r2.velocity_x= 0.0;
    //    r2.velocity_y= 0.0;
    //    r2.velocity_z=0.0;
    //    r2.radius =1.0;
    //    r2.nitro_amount=0.0;
    //    r2.touch_normal_x=0.0;
    //    r2.touch_normal_y = 1.0;
    //    r2.touch_normal_z=0.0;
    //    r2.touch = true;
    //    r2.is_teammate = true;
    //    game->robots.push_back(r2);
    //
    //    Robot r3 = Robot();
    //    r3.id = 3;
    //    r3.player_id = 1;
    //    r3.x= -7.856392606191905;
    //    r3.y =1.0;
    //    r3.z=18.39231076339711;
    //    r3.velocity_x= 0.0;
    //    r3.velocity_y= 0.0;
    //    r3.velocity_z=0.0;
    //    r3.radius =1.0;
    //    r3.nitro_amount=0.0;
    //    r3.touch_normal_x=0.0;
    //    r3.touch_normal_y = 1.0;
    //    r3.touch_normal_z=0.0;
    //    r3.touch = true;
    //    game->robots.push_back(r3);
    //
    //    Robot r4 = Robot();
    //    r4.id = 4;
    //    r4.player_id = 1;
    //    r4.x = 12.000012052303903;
    //    r4.y = 1.0;
    //    r4.z = 15.99999096076498;
    //    r4.velocity_x= 0.0;
    //    r4.velocity_y= 0.0;
    //    r4.velocity_z=0.0;
    //    r4.radius =1.0;
    //    r4.nitro_amount=0.0;
    //    r4.touch_normal_x=0.0;
    //    r4.touch_normal_y = 1.0;
    //    r4.touch_normal_z=0.0;
    //    r4.touch = true;
    //    game->robots.push_back(r4);
    //
    //    RoleParameters gk;
    //    vector<RoleParameters> forwards;
    //
        sim.init(*game, *rules, 1);
        game->current_tick=1;
        sim.setTick(*game);


//    blabla(game->ball, r1, rules.get());


    cout << " start speed " << rules->ROBOT_MAX_JUMP_SPEED << endl;
    double time = rules->ROBOT_MAX_JUMP_SPEED / rules->GRAVITY;
    cout << " fly time " << time << endl;
    double ticks = time * rules->TICKS_PER_SECOND;
    cout << "ticks count " << ticks << endl;
    double height = rules->GRAVITY * time * time / 2.0;
    cout << " height " << height << endl;



//    e.position.setY(e.position.getY() - rules.GRAVITY * delta_time * delta_time / 2.0);
//    e.velocity.setY(e.velocity.getY() - rules.GRAVITY * delta_time);


    double dt = 1.0 / (rules->TICKS_PER_SECOND);// * rules->MICROTICKS_PER_TICK);
    double testHeight = 0.05;
    double vel = rules->ROBOT_MAX_JUMP_SPEED;
    for (int j = 0; j < ticks; j++) {
//        for (int i = 0; i < rules->MICROTICKS_PER_TICK; i++) {
            testHeight += vel * dt - rules->GRAVITY * dt * dt / 2;
            vel -= rules->GRAVITY * dt;
//        }
        cout << "text height " << testHeight << " on tick " << j<< endl;
    }
    
    SimulationEntity se;
    se.setPosition(Vec3(0.00, 1.00, -39.792046));
    se.setVelocity(Vec3(0,0,0));
   sim.checkAchievement(se
                        , Vec3(-0.062784, 1.030923, -39.792046)
                        , Vec3(-0.062784, 2.030923, -39.792046)
                        , 12);
}
/*text height 0.295833
text height 0.533333
text height 0.7625
text height 0.983333
text height 1.19583
text height 1.4
text height 1.59583
text height 1.78333
text height 1.9625
text height 2.13333
text height 2.29583
text height 2.45
text height 2.59583
text height 2.73333
text height 2.8625
text height 2.98333
text height 3.09583
text height 3.2
text height 3.29583
text height 3.38333
text height 3.4625
text height 3.53333
text height 3.59583
text height 3.65
text height 3.69583
text height 3.73333
text height 3.7625
text height 3.78333
text height 3.79583
text height 3.8*/
int main(int argc, char *argv[]) {
//        if (argc == 4) {
//            Runner runner(argv[1], argv[2], argv[3]);
//            runner.run();
//        } else {
//            Runner runner("127.0.0.1", "31001", "0000000000000000");
//            runner.run();
//        }

    testRun1();
    return 0;
}

Runner::Runner(const char *host, const char *port, const char *token)
        : remoteProcessClient(host, atoi(port)), token(token) {
}

void Runner::run() {
    unique_ptr<Strategy> strategy(new MyStrategy);
    unique_ptr<Game> game;
    unordered_map<int, Action> actions;
    remoteProcessClient.write_token(token);
    unique_ptr<Rules> rules = remoteProcessClient.read_rules();
    while ((game = remoteProcessClient.read_game()) != nullptr) {
        actions.clear();
        for (const Robot &robot : game->robots) {
            if (robot.is_teammate) {
                strategy->act(robot, *rules, *game, actions[robot.id]);
            }
        }
        remoteProcessClient.write(actions, strategy->custom_rendering());
    }
}

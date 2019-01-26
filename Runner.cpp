#include <memory>

#include "Runner.h"
#include "MyStrategy.h"
#include <iomanip>

using namespace model;
using namespace std;


void blabla(Ball ball, Robot r1, Rules *rules, Simulation& sim) {
    SimulationEntity seBall;
    seBall.velocity = Vec3(ball.velocity_x, ball.velocity_y, ball.velocity_z);
    seBall.position = Vec3(ball.x, ball.y, ball.z);

    SimulationEntity rob;
    rob.velocity = Vec3(r1.velocity_x, r1.velocity_y, r1.velocity_z);
    rob.touch_normal = Vec3(r1.touch_normal_x, r1.touch_normal_y, r1.touch_normal_z);
    rob.position = Vec3(r1.x, r1.y, r1.z);

    int first_attempt = 46;

    cout << " Ball initial position "<< seBall.position.toString()<<endl;
    Vec3 bptarget = sim.getHitPosition(seBall, rob);
    cout << " Hit position "<< bptarget.toString() << endl;
    for ( ; first_attempt < 60; first_attempt++){
        vector<SimulationEntity> route;
        if (sim.checkAchievement(rob, bptarget, first_attempt, route)){
            cout << endl << " Ticks enough "<< first_attempt << endl << endl;
            return;
        }
    }
    
}

double jumpSpeed(double targetHeight, Rules * rules){
    double dt = 1.0 / (rules->TICKS_PER_SECOND);
    targetHeight = targetHeight - rules->ROBOT_RADIUS;
    cout<< "Target height "<< targetHeight<<endl;
    
    double targetTime = sqrt(targetHeight * 2.0/ rules->GRAVITY);
    cout << " Target time " << targetTime << endl;
    
    if (targetTime > 30){
        
    }

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
    game->ball.y = 3.59698600000018;
    game->ball.z = 0.0;
    game->ball.velocity_x = 1.0;
    game->ball.velocity_y = 0.0;
    game->ball.velocity_z = 1.0;
    game->ball.radius = 2.0;

    game->current_tick = 0;
    Robot r1 = Robot();
    r1.id = 1;
    r1.player_id = 0;
    r1.x = 13.78179241085154;
    r1.y = 1.0;
    r1.z = -14.493522620267125;
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


    //blabla(game->ball,r1, rules.get(), sim);
    vector<SimulationEntity> route;
    SimulationEntity rr1;
    rr1.setNormal(Vec3(0,1,0));
    rr1.touch = true;
    rr1.setVelocity(Vec3(0,0,-10));
    rr1.setPosition(Vec3(0, 1, 0));

    sim.checkAchievement(rr1, Vec3(0, 1, -10),  25, route);


    /*## Check achievement for robot on position (-22.551581;1.000000;7.906551) size=23.918348
     velocity (-15.998261;0.000000;-25.378251) size=30.000000
     and bp target (-24.305205;1.000000;0.320872) size=24.327884
     Available delta time 44
     Jump ticks 0
     Full jump time in sec 0
     ##_>> INITIAL DELTA (-1.753624;0.000000;-7.585679) size=7.785738
     rr1.velocity (-15.998261;0.000000;-25.378251) size=30.000000
     Jp.fullTimeSpec 0 attempts_t 0.733333 delta time 0.000166667
     Right part time 0.148751
     New acceleration time 0
     New acceleration time 0.183333
     New acceleration time 0.366667
     New acceleration time 0.275
     New acceleration time 0.183333
     New acceleration time 0.229167
     New acceleration time 0.275
     New acceleration time 0.252083
     New acceleration time 0.229167
     New acceleration time 0.240625
     New acceleration time 0.252083
     New acceleration time 0.246354
     New acceleration time 0.240625
     New acceleration time 0.24349
     New acceleration time 0.242057
     New acceleration time 0.242773
     New acceleration time 0.24349
     New acceleration time 0.243132
     New acceleration time 0.243311
     New acceleration time 0.243221
     Velocity final (-21.476460;0.000000;-49.075383) size=53.568942
     Too big velocity
     Dist_K 2.82843
     Ball velocity (0.117183;-0.407042;-0.386446) size=0.573373
     Ball position (-24.188021;2.233307;2.448965) size=24.414043
     Goal direction (24.188021;5.766693;41.551035) size=48.423173
     Goal direction (0.503094;0.000000;0.864232) size=1.000000
     Goal direction (0.385910;0.000000;1.250678) size=1.308863
     Goal direction (0.294844;0.000000;0.955545) size=1.000000
     Goal direction (0.855047;0.000000;2.771082) size=2.900000
     hit position' (-24.498940;1.598629;1.441323) size=24.593314
     Initial hit position (-24.498940;1.598629;1.441323) size=24.593314
     Hit position (-24.498940;1.598629;1.441323) size=24.593314 for tick 3986
*/
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
        if (argc == 4) {
            Runner runner(argv[1], argv[2], argv[3]);
            runner.run();
        } else {
            Runner runner("127.0.0.1", "31001", "0000000000000000");
            runner.run();
        }

//    testRun1();
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

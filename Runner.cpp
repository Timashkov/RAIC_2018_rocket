#include <memory>

#include "Runner.h"
#include "MyStrategy.h"
#include <iomanip>

using namespace model;
using namespace std;

void testRun1(){
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
    rules->MICROTICKS_PER_TICK= 100;
    rules->RESET_TICKS=120;
    rules->BALL_ARENA_E=0.7;
    rules->BALL_RADIUS=2.0;
    rules->BALL_MASS=1.0;
    rules->MIN_HIT_E=0.4;
    rules->MAX_HIT_E=0.5;
    rules->MAX_ENTITY_SPEED=100.0;
    rules->MAX_NITRO_AMOUNT=100.0;
    rules->START_NITRO_AMOUNT=50.0;
    rules->NITRO_POINT_VELOCITY_CHANGE=0.6;
    rules->NITRO_PACK_X=20.0;
    rules->NITRO_PACK_Y=1.0;
    rules->NITRO_PACK_Z=30.0;
    rules->NITRO_PACK_RADIUS=0.5;
    rules->NITRO_PACK_AMOUNT=100.0;
    rules->NITRO_PACK_RESPAWN_TICKS=600;
    rules->GRAVITY=30.0;

    game->ball = Ball();
    game->ball.x = 0.0;
    game->ball.y = 6.092095;
    game->ball.z = 0.0;
    game->ball.velocity_x = 0.0;
    game->ball.velocity_y = 0.0;
    game->ball.velocity_z = 0.0;
    game->ball.radius = 2.0;
    
    game->current_tick = 0;
    Robot r1 = Robot();
    r1.id = 1;
    r1.player_id = 0;
    r1.x= 7.856392606191905;
    r1.y =1.0;
    r1.z=-18.39231076339711;
    r1.velocity_x= 0.0;
    r1.velocity_y= 0.0;
    r1.velocity_z=0.0;
    r1.radius =1.0;
    r1.nitro_amount=0.0;
    r1.touch_normal_x=0.0;
    r1.touch_normal_y = 1.0;
    r1.touch_normal_z=0.0;
    r1.touch = true;
    r1.is_teammate = true;
    game->robots.push_back(r1);
    
    Robot r2 = Robot();
    r2.id = 2;
    r2.player_id = 0;
    r2.x= -5.860171;
    r2.y =1.0;
    r2.z=-19.122196;
    r2.velocity_x= 0.0;
    r2.velocity_y= 0.0;
    r2.velocity_z=0.0;
    r2.radius =1.0;
    r2.nitro_amount=0.0;
    r2.touch_normal_x=0.0;
    r2.touch_normal_y = 1.0;
    r2.touch_normal_z=0.0;
    r2.touch = true;
    r2.is_teammate = true;
    game->robots.push_back(r2);
    
    Robot r3 = Robot();
    r3.id = 3;
    r3.player_id = 1;
    r3.x= -7.856392606191905;
    r3.y =1.0;
    r3.z=18.39231076339711;
    r3.velocity_x= 0.0;
    r3.velocity_y= 0.0;
    r3.velocity_z=0.0;
    r3.radius =1.0;
    r3.nitro_amount=0.0;
    r3.touch_normal_x=0.0;
    r3.touch_normal_y = 1.0;
    r3.touch_normal_z=0.0;
    r3.touch = true;
    game->robots.push_back(r3);
    
    Robot r4 = Robot();
    r4.id = 4;
    r4.player_id = 1;
    r4.x = 12.000012052303903;
    r4.y = 1.0;
    r4.z = 15.99999096076498;
    r4.velocity_x= 0.0;
    r4.velocity_y= 0.0;
    r4.velocity_z=0.0;
    r4.radius =1.0;
    r4.nitro_amount=0.0;
    r4.touch_normal_x=0.0;
    r4.touch_normal_y = 1.0;
    r4.touch_normal_z=0.0;
    r4.touch = true;
    game->robots.push_back(r4);
    
    RoleParameters gk;
    vector<RoleParameters> forwards;
    
    sim.init(*game, *rules, gk, forwards);
    game->current_tick=1;
    sim.setTick(*game);

}

int main(int argc, char* argv[]) {
//    if (argc == 4) {
//        Runner runner(argv[1], argv[2], argv[3]);
//        runner.run();
//    } else {
//        Runner runner("127.0.0.1", "31001", "0000000000000000");
//        runner.run();
//    }
    
        testRun1();
    return 0;
}

Runner::Runner(const char* host, const char* port, const char* token)
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
        for (const Robot& robot : game->robots) {
            if (robot.is_teammate) {
                strategy->act(robot, *rules, *game, actions[robot.id]);
            }
        }
        remoteProcessClient.write(actions, strategy->custom_rendering());
    }
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
*/

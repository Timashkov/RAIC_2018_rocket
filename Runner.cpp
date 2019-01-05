#include <memory>

#include "Runner.h"
#include "MyStrategy.h"
#include <iomanip>

using namespace model;
using namespace std;

void testRun(){
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
    game->ball.y = 7.826750919657936;
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
    game->robots.push_back(r1);
    
    Robot r2 = Robot();
    r2.id = 2;
    r2.player_id = 0;
    r2.x= -12.000012052303903;
    r2.y =1.0;
    r2.z=-15.99999096076498;
    r2.velocity_x= 0.0;
    r2.velocity_y= 0.0;
    r2.velocity_z=0.0;
    r2.radius =1.0;
    r2.nitro_amount=0.0;
    r2.touch_normal_x=0.0;
    r2.touch_normal_y = 1.0;
    r2.touch_normal_z=0.0;
    r2.touch = true;
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
//    sim.start();
}

int main(int argc, char* argv[]) {
    if (argc == 4) {
        Runner runner(argv[1], argv[2], argv[3]);
        runner.run();
    } else {
        Runner runner("127.0.0.1", "31001", "0000000000000000");
        runner.run();
    }
    
//        testRun1();
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

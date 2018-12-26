#include <memory>

//
//  Simulation.cpp
//  raic
//
//  Created by Alex on 23/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "Simulation.h"
#include "CVL_Utils.h"


Dan Simulation::dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal) {
    return Dan(dot((point - point_on_plane), plane_normal), plane_normal);
}

Dan Simulation::dan_to_sphere_inner(Vec3 point, Vec3 sphere_center, double sphere_radius) {
    return Dan(sphere_radius - (point - sphere_center).len(), (sphere_center - point).normalized());
}

Dan Simulation::dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius) {
    return Dan((point - sphere_center).len() - sphere_radius, (point - sphere_center).normalized());
}

Dan Simulation::min(Dan a, Dan b) {
    return a.distance < b.distance ? a : b;
}

Dan Simulation::dan_to_arena_quarter(Vec3 point) {    // Ground
    Dan dan = dan_to_plane(point, Vec3(0.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));

    // Ceiling
    dan = min(dan, dan_to_plane(point, Vec3(0.0, arena.height, 0.0), Vec3(0.0, -1.0, 0.0)));

    // Side x
    dan = min(dan, dan_to_plane(point, Vec3(arena.width / 2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));

    // Side z (goal)
    dan = min(dan, dan_to_plane(point, Vec3(0.0, 0.0, (arena.depth / 2.0) + arena.goal_depth), Vec3(0.0, 0.0, -1.0)));

    // Side z
    Vec3 v = Vec3(point.getX(), point.getY(), 0.0) -
             (Vec3((arena.goal_width / 2.0) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius, 0.0));
    if (point.getX() >= (arena.goal_width / 2.0) + arena.goal_side_radius
        || point.getY() >= arena.goal_height + arena.goal_side_radius || (
                v.getX() > 0.0
                && v.getY() > 0.0
                && v.len() >= arena.goal_top_radius + arena.goal_side_radius)) {
        dan = min(dan, dan_to_plane(point, Vec3(0.0, 0.0, arena.depth / 2.0), Vec3(0.0, 0.0, -1.0)));
    }

    // Side x & ceiling (goal)
    if (point.getZ() >= (arena.depth / 2.0) + arena.goal_side_radius) {
        // x
        dan = min(dan, dan_to_plane(point, Vec3(arena.goal_width / 2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
        // y
        dan = min(dan, dan_to_plane(point, Vec3(0.0, arena.goal_height, 0.0), Vec3(0.0, -1.0, 0.0)));
    }

    // Goal back corners
    //    assert arena.bottom_radius == arena.goal_top_radius
    if (point.getZ() > (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius) {
        dan = min(dan, dan_to_sphere_inner(point,
                                           Vec3(clamp(point.getX(), arena.bottom_radius - (arena.goal_width / 2.0),
                                                      (arena.goal_width / 2.0) - arena.bottom_radius),
                                                clamp(point.getY(),
                                                      arena.bottom_radius,
                                                      arena.goal_height - arena.goal_top_radius),
                                                (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius),
                                           arena.bottom_radius));
    }
    // Corner
    if (point.getX() > (arena.width / 2.0) - arena.corner_radius
        and point.getZ() > (arena.depth / 2.0) - arena.corner_radius) {
        dan = min(dan, dan_to_sphere_inner(
                point,
                Vec3(
                        (arena.width / 2.0) - arena.corner_radius,
                        point.getY(),
                        (arena.depth / 2.0) - arena.corner_radius
                ),
                arena.corner_radius));
    }
    // Goal outer corner
    if (point.getZ() < (arena.depth / 2.0) + arena.goal_side_radius) {
        // Side x
        if (point.getX() < (arena.goal_width / 2.0) + arena.goal_side_radius) {
            dan = min(dan, dan_to_sphere_outer(
                    point,
                    Vec3(
                            (arena.goal_width / 2.0) + arena.goal_side_radius,
                            point.getY(),
                            (arena.depth / 2.0) + arena.goal_side_radius
                    ),
                    arena.goal_side_radius));

        }
        // Ceiling
        if (point.getY() < arena.goal_height + arena.goal_side_radius) {
            dan = min(dan, dan_to_sphere_outer(
                    point,
                    Vec3(
                            point.getX(),
                            arena.goal_height + arena.goal_side_radius,
                            (arena.depth / 2.0) + arena.goal_side_radius
                    ),
                    arena.goal_side_radius));
        }
        // Top corner
        Vec3 o1 = Vec3((arena.goal_width / 2.0) - arena.goal_top_radius,
                       arena.goal_height - arena.goal_top_radius, 0.0);
        Vec3 v = Vec3(point.getX(), point.getY(), 0.0) - o1;
        if (v.getX() > 0.0 && v.getY() > 0.0) {
            Vec3 o = o1 + v.normalized() * (arena.goal_top_radius + arena.goal_side_radius);
            dan = min(dan, dan_to_sphere_outer(
                    point,
                    Vec3(o.getX(), o.getY(), (arena.depth / 2.0) + arena.goal_side_radius),
                    arena.goal_side_radius));

        }
    }
    // Goal inside top corners
    if (point.getZ() > (arena.depth / 2.0) + arena.goal_side_radius
        && point.getY() > arena.goal_height - arena.goal_top_radius) {
        // Side x
        if (point.getX() > (arena.goal_width / 2.0) - arena.goal_top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3(
                            (arena.goal_width / 2.0) - arena.goal_top_radius,
                            arena.goal_height - arena.goal_top_radius,
                            point.getZ()
                    ),
                    arena.goal_top_radius));

        }
        // Side z
        if (point.getZ() > (arena.depth / 2.0) + arena.goal_depth - arena.goal_top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3(
                            point.getX(),
                            arena.goal_height - arena.goal_top_radius,
                            (arena.depth / 2.0) + arena.goal_depth - arena.goal_top_radius
                    ),
                    arena.goal_top_radius));
        }
    }
    // Bottom corners
    if (point.getY() < arena.bottom_radius) {
        // Side x
        if (point.getX() > (arena.width / 2.0) - arena.bottom_radius) {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3(
                            (arena.width / 2.0) - arena.bottom_radius,
                            arena.bottom_radius,
                            point.getZ()
                    ),
                    arena.bottom_radius));
        }
        // Side z
        if (point.getZ() > (arena.depth / 2.0) - arena.bottom_radius
            and point.getX() >= (arena.goal_width / 2.0) + arena.goal_side_radius) {
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                       point.getX(),
                                                       arena.bottom_radius,
                                                       (arena.depth / 2.0) - arena.bottom_radius
                                               ),
                                               arena.bottom_radius));
        }
        // Side z (goal)
        if (point.getZ() > (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius) {
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                       point.getX(),
                                                       arena.bottom_radius,
                                                       (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius
                                               ),
                                               arena.bottom_radius));
        }
        // Goal outer corner
        Vec3 o1 = Vec3((arena.goal_width / 2.0) + arena.goal_side_radius,
                       (arena.depth / 2.0) + arena.goal_side_radius, 0.0);
        Vec3 v = Vec3(point.getX(), point.getZ(), 0.0) - o1;
        if (v.getX() < 0.0 && v.getY() < 0.0 && v.len() < arena.goal_side_radius + arena.bottom_radius) {
            Vec3 o = o1 + v.normalized() * (arena.goal_side_radius + arena.bottom_radius);
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(o.getX(), arena.bottom_radius, o.getY()),
                                               arena.bottom_radius));

        }
        // Side x (goal)
        if (point.getZ() >= (arena.depth / 2.0) + arena.goal_side_radius
            and point.getX() > (arena.goal_width / 2.0) - arena.bottom_radius) {
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                       (arena.goal_width / 2.0) - arena.bottom_radius,
                                                       arena.bottom_radius, point.getZ()),
                                               arena.bottom_radius));

        }
        // Corner
        if (point.getX() > (arena.width / 2.0) - arena.corner_radius
            and point.getZ() > (arena.depth / 2.0) - arena.corner_radius) {
            Vec3 corner_o = Vec3((arena.width / 2.0) - arena.corner_radius,
                                 (arena.depth / 2.0) - arena.corner_radius, 0.0);
            Vec3 n = Vec3(point.getX(), point.getZ(), 0.0) - corner_o;
            double dist = n.len();
            if (dist > arena.corner_radius - arena.bottom_radius) {
                n = n / dist;
                Vec3 o2 = corner_o + n * (arena.corner_radius - arena.bottom_radius);
                dan = min(dan, dan_to_sphere_inner(
                        point,
                        Vec3(o2.getX(), arena.bottom_radius, o2.getY()),
                        arena.bottom_radius));
            }
        }
    }
    // Ceiling corners
    if (point.getY() > arena.height - arena.top_radius) {
        // Side x
        if (point.getX() > (arena.width / 2.0) - arena.top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3(
                            (arena.width / 2.0) - arena.top_radius,
                            arena.height - arena.top_radius,
                            point.getZ()
                    ),
                    arena.top_radius));

        }
        // Side z
        if (point.getZ() > (arena.depth / 2.0) - arena.top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3(
                            point.getX(),
                            arena.height - arena.top_radius,
                            (arena.depth / 2.0) - arena.top_radius
                    ),
                    arena.top_radius));

        }

        // Corner
        if (point.getX() > (arena.width / 2.0) - arena.corner_radius
            && point.getZ() > (arena.depth / 2.0) - arena.corner_radius) {
            Vec3 corner_o = Vec3((arena.width / 2.0) - arena.corner_radius,
                                 (arena.depth / 2.0) - arena.corner_radius, 0.0);
            Vec3 dv = Vec3(point.getX(), point.getZ(), 0.0) - corner_o;
            if (dv.len() > arena.corner_radius - arena.top_radius) {
                Vec3 n = dv.normalized();
                Vec3 o2 = corner_o + n * (arena.corner_radius - arena.top_radius);
                dan = min(dan, dan_to_sphere_inner(
                        point,
                        Vec3(o2.getX(), arena.height - arena.top_radius, o2.getY()),
                        arena.top_radius));
            }
        }
    }
    return dan;
}

Dan Simulation::dan_to_arena(Vec3 &point) {
    bool negate_x = point.getX() < 0.0;
    bool negate_z = point.getZ() < 0.0;
    if (negate_x) {
        point.setX(-point.getX());
    }
    if (negate_z) {
        point.setZ(-point.getZ());
    }
    Dan result = dan_to_arena_quarter(point);
    if (negate_x) {
        result.normal.setX(-result.normal.getX());
    }
    if (negate_z) {
        result.normal.setZ(-result.normal.getZ());
    }
    return result;
}

// Направление определено жестко, рандом только в скорости после удара

void Simulation::collide_entities(Entity &a, Entity &b) {
    Vec3 delta_position = b.position - a.position;
    double distance = delta_position.len();
    double penetration = a.radius + b.radius - distance;
    if (penetration > 0.0) {
        double k_a = (1.0 / a.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
        double k_b = (1.0 / b.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
        Vec3 normal = delta_position.normalized();
        a.setPosition(a.position - (normal * penetration * k_a));
        b.setPosition(b.position + (normal * penetration * k_b));
        Vec3 velodelta = b.velocity - a.velocity;
        double delta_velocity = dot(velodelta, normal) + b.radius_change_speed - a.radius_change_speed;
        if (delta_velocity < 0.0) {
            Vec3 impulse = normal * (1.0 + random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity;
            a.velocity = a.velocity + (impulse * k_a);
            b.velocity = b.velocity - (impulse * k_b);
        }
    }
}

Vec3 Simulation::collide_with_arena(Entity &e) {

    Dan danArena = dan_to_arena(e.position);
    if (e.id == tester_id) cout<< "Dan to arena " << danArena.distance << "  " << danArena.normal.toString() << endl;
    double penetration = e.radius - danArena.distance;
    if (penetration > 0.0) {
        e.position = e.position + danArena.normal * penetration;
        double velocity = dot(e.velocity, danArena.normal) - e.radius_change_speed;
        if (velocity < 0.0) {
            e.velocity = e.velocity - danArena.normal * velocity * (1.0 + rules.BALL_ARENA_E);
            return danArena.normal;
        }
    }
    return Vec3::None;
}

void Simulation::move(Entity &e, double delta_time) {
    e.velocity = clamp(e.velocity, rules.MAX_ENTITY_SPEED);
    e.position = e.position + e.velocity * delta_time;
    e.position.setY(e.position.getY() - rules.GRAVITY * delta_time * delta_time / 2.0);
    e.velocity.setY(e.velocity.getY() - rules.GRAVITY * delta_time);
}

void Simulation::update(shared_ptr<TreeNode> &node, double delta_time) {
    // FIXME: after investigations
    //    shuffle(robots);

    moveRobots(node, delta_time);
    move(node->state.ball, delta_time);

    for (int i = 0; i < node->state.robots.size(); i++) {
        for (int j = 0; j < i - 1; j++) {
            collide_entities(node->state.robots[i], node->state.robots[j]);
        }
    }

    for (Entity &robot : node->state.robots) {
        collide_entities(robot, node->state.ball);
        Vec3 collision_normal = collide_with_arena(robot);
        if (collision_normal == Vec3::None) {
            robot.touch = false;
        } else {
            robot.touch = true;
            robot.touch_normal = collision_normal;
        }
    }
    collide_with_arena(node->state.ball);

    if (abs(node->state.ball.position.getZ()) > arena.depth / 2.0 + node->state.ball.radius) {
        goal_scored();
    }

    for (Entity &robot : node->state.robots) {
        if (robot.nitro == rules.MAX_NITRO_AMOUNT)
            continue;
        for (Entity &pack : node->state.nitro_packs) {
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
    for (Entity &robot : node->state.robots) {
        if (robot.id == tester_id)  cout << "Robot id" << robot.id << " touch " << robot.touch << endl;
        Vec3 action_target_velocity = Vec3(robot.action.target_velocity_x, robot.action.target_velocity_y,
                                           robot.action.target_velocity_z);
        if (robot.id == tester_id)cout << "0::  " << robot.action.target_velocity_z << " current vz "
                                     << robot.velocity.getZ() << " Z pos: " << robot.position.getZ() << endl;
        if (robot.touch) {
            Vec3 target_velocity = clamp(action_target_velocity, rules.ROBOT_MAX_GROUND_SPEED);
            if (robot.id == tester_id) cout << "1::  " << target_velocity.toString() << endl;
            target_velocity = target_velocity - (robot.touch_normal * dot(robot.touch_normal, target_velocity));
            if (robot.id == tester_id)cout << "2::  " << target_velocity.toString() << endl;
            Vec3 target_velocity_change = target_velocity - robot.velocity;
            if (robot.id == tester_id)cout << "3::  " << target_velocity_change.toString() << endl;
            if (target_velocity_change.len() > 0.0) {
                if (robot.id == tester_id)cout << "4::  " << target_velocity_change.len() << endl;
                double acceleration = rules.ROBOT_ACCELERATION * max(0.0, robot.touch_normal.getY());
                if (robot.id == tester_id)cout << "5::  " << robot.touch_normal.getY() << endl;
                robot.velocity = robot.velocity + clamp(target_velocity_change.normalized() * acceleration * delta_time,
                                                        target_velocity_change.len());
                if (robot.id == tester_id)cout << "Action new v Z " << robot.velocity.getZ() << endl;
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
        move(robot, delta_time);
        robot.radius = rules.ROBOT_MIN_RADIUS +
                       (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed /
                       rules.ROBOT_MAX_JUMP_SPEED;
        robot.radius_change_speed = robot.action.jump_speed;
    }
}

void Simulation::tick(shared_ptr<TreeNode> parent) {
    double delta_time = 1.0 / rules.TICKS_PER_SECOND;
    double micro_dt = delta_time / rules.MICROTICKS_PER_TICK;

    State st = State(parent->state);
    st.current_tick++;

    for (Entity &rob: st.robots) {
        if (rob.id == tester_id) {
            cout << "ROB Velocity: " << rob.velocity.toString() << " position:" << rob.position.toString() << endl;
            Vec3 target_pos = Vec3(rob.position.getZ(), 0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius);
            Vec3 target_velocity = Vec3(rob.position.getX(), 0.0, target_pos.getZ() - rob.position.getZ()).mul(
                    rules.ROBOT_MAX_GROUND_SPEED);
            rob.action.target_velocity_x = target_velocity.getX();
            rob.action.target_velocity_y = target_velocity.getY();
            rob.action.target_velocity_z = target_velocity.getZ();
            rob.action.jump_speed = 0.0;
            rob.action.use_nitro = false;
        }
    }

    if (st.current_tick > 10 + current_tick)
        return;

    shared_ptr<TreeNode> node = make_shared<TreeNode>(st, parent.get());
    parent->children.push_back(node);

    for (int i = 0; i < rules.MICROTICKS_PER_TICK; i++) {
        update(node, micro_dt);
    }

    for (Entity pack : node->state.nitro_packs) {
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
        cout << "2::" << tn->state.current_tick << endl;
        processingNodes.pop();
        tick(tn);
    }
}

void Simulation::init(const Game &g, const Rules &rul) {
    cout << "Init" << endl;
    rules = rul;
    std::srand(rul.seed);

    State st;
    st.ball.setPosition(g.ball.x, g.ball.y, g.ball.z);
    st.ball.radius = g.ball.radius;
    st.ball.mass = rul.BALL_MASS;
    st.current_tick = 0;

    arena = rul.arena;

    if (tester_id == -1) {
        int k = 1000;
        for (Robot r: g.robots) {
            if (r.id < k)
                k = r.id;
        }
        tester_id = k;
        cout << "TesterId :" << tester_id << endl;
    }

    for (Robot rob: g.robots) {
        Entity erob;
        erob.id = rob.id;
        erob.player_id = rob.player_id;
        erob.setPosition(rob.x, rob.y, rob.z);
        erob.setVelocity(rob.velocity_x, rob.velocity_y, rob.velocity_z);
        erob.setNormal(rob.touch_normal_x, rob.touch_normal_y, rob.touch_normal_z);
        erob.touch = rob.touch;
        erob.mass = rul.ROBOT_MASS;
        erob.radius = rul.ROBOT_RADIUS;

        if (rob.id == tester_id) {
            cout << "ROB VZ: " << rob.velocity_z << " Z:" << rob.z << endl;
            Vec3 target_pos = Vec3(rob.x, 0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius);
            Vec3 target_velocity = Vec3(rob.x, 0.0, target_pos.getZ() - rob.z).mul(rules.ROBOT_MAX_GROUND_SPEED);
            cout << "Target velocity " << target_velocity.toString() << endl;
            erob.action.target_velocity_x = target_velocity.getX();
            erob.action.target_velocity_y = target_velocity.getY();
            erob.action.target_velocity_z = target_velocity.getZ();
            erob.action.jump_speed = 0.0;
            erob.action.use_nitro = false;
        }

        st.robots.push_back(erob);
    }

    baseNode = std::make_shared<TreeNode>(st, nullptr);

    dumpNode(baseNode);
    inited = true;
    cout << "inited" << endl;
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

    cout << "hz" << endl;
#ifdef LOCAL_RUN
    ss << " Simulation done " << endl;

    writeLog(ss);
    std::stringstream().swap(ss);
#endif
}

void Simulation::dumpNode(shared_ptr<TreeNode> node) {
#ifdef LOCAL_RUN
    std::stringstream ss;
    ss << " simulaton:: tick : " << node->state.current_tick << endl;
    ss << " BALL radius:" << node->state.ball.radius;
    ss << " coord:(" << node->state.ball.position.getX() << ";" << node->state.ball.position.getY() << ";"
       << node->state.ball.position.getZ() << ")";
    ss << " velocity:(" << node->state.ball.velocity.getX() << ";" << node->state.ball.velocity.getY() << ";"
       << node->state.ball.velocity.getZ() << ")"
       << std::endl;

    for (Entity r: node->state.robots) {
        ss << " ROBOT: id: *" << r.id << "* " << "* ";
        ss << " player_id: *" << r.player_id << "*";
        ss << " coord: (" << r.position.getX() << ";" << r.position.getY() << ":" << r.position.getZ() << ")";
        ss << " velocity: (" << r.velocity.getX() << ";" << r.velocity.getY() << ";" << r.velocity.getZ() << ")";
        ss << " radius:" << r.radius;
        ss << " nitro:" << r.nitro;
        ss << " touch:" << r.touch;
        ss << " touch_normal: x:" << r.touch_normal.getX() << "Y:" << r.touch_normal.getY() << "z:"
           << r.touch_normal.getZ() << std::endl << std::endl;
    }

    writeLog(ss);
    std::stringstream().swap(ss);
#endif
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

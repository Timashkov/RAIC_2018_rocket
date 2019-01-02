//
//  SimulationEngine.cpp
//  raic
//
//  Created by Alex on 31/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "SimulationEngine.h"
#include <iostream>

void SimulationEngine::move(SimulationEntity &e, double delta_time) {
    e.velocity = clamp(e.velocity, rules.MAX_ENTITY_SPEED);
    e.position = e.position + e.velocity * delta_time;
    e.position.setY(e.position.getY() - rules.GRAVITY * delta_time * delta_time / 2.0);
    e.velocity.setY(e.velocity.getY() - rules.GRAVITY * delta_time);
}

// Направление определено жестко, рандом только в скорости после удара
bool SimulationEngine::collide_entities(SimulationEntity &a, SimulationEntity &b) {
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
        return true;
    }
    return false;
}

Vec3 SimulationEngine::collide_with_arena(SimulationEntity &e) {
    
    Dan danArena = dan_to_arena(e.position);
    double penetration = e.radius - danArena.distance;
    if (penetration > 0.0) {
        e.position = e.position + danArena.normal * penetration;
        double velocity = dot(e.velocity, danArena.normal) - e.radius_change_speed;
        if (velocity < 0.0) {
            e.velocity = e.velocity - danArena.normal * velocity * (1.0 + e.arena_e);
            return danArena.normal;
        }
    }
    return Vec3::None;
}

void SimulationEngine::moveRobots(vector<SimulationEntity>& robots, double delta_time) {
    for (SimulationEntity &robot : robots) {
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
        move(robot, delta_time);
        robot.radius = rules.ROBOT_MIN_RADIUS +
                       (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed /
                       rules.ROBOT_MAX_JUMP_SPEED;
        robot.radius_change_speed = robot.action.jump_speed;
    }
}

Dan SimulationEngine::dan_to_arena(Vec3 point) {
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

Dan SimulationEngine::dan_to_arena_quarter(Vec3 point) {
    // Ground
    Dan dan = dan_to_plane(point, Vec3(0.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
    
    // Ceiling
    dan = min(dan, dan_to_plane(point, Vec3(0.0, rules.arena.height, 0.0), Vec3(0.0, -1.0, 0.0)));
    
    // Side x
    dan = min(dan, dan_to_plane(point, Vec3(rules.arena.width / 2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
    
    // Side z (goal)
    dan = min(dan, dan_to_plane(point, Vec3(0.0, 0.0, (rules.arena.depth / 2.0) + rules.arena.goal_depth), Vec3(0.0, 0.0, -1.0)));
    
    // Side z
    Vec3 v = Vec3(point.getX(), point.getY(), 0.0) -
    (Vec3((rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius, rules.arena.goal_height - rules.arena.goal_top_radius, 0.0));
    if (point.getX() >= (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius
        || point.getY() >= rules.arena.goal_height + rules.arena.goal_side_radius || (
                                                                          v.getX() > 0.0
                                                                          && v.getY() > 0.0
                                                                          && v.len() >= rules.arena.goal_top_radius + rules.arena.goal_side_radius)) {
            dan = min(dan, dan_to_plane(point, Vec3(0.0, 0.0, rules.arena.depth / 2.0), Vec3(0.0, 0.0, -1.0)));
        }
    
    // Side x & ceiling (goal)
    if (point.getZ() >= (rules.arena.depth / 2.0) + rules.arena.goal_side_radius) {
        // x
        dan = min(dan, dan_to_plane(point, Vec3(rules.arena.goal_width / 2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
        // y
        dan = min(dan, dan_to_plane(point, Vec3(0.0, rules.arena.goal_height, 0.0), Vec3(0.0, -1.0, 0.0)));
    }
    
    // Goal back corners
    //    assert arena.bottom_radius == arena.goal_top_radius
    if (point.getZ() > (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius) {
        dan = min(dan, dan_to_sphere_inner(point,
                                           Vec3(clamp(point.getX(), rules.arena.bottom_radius - (rules.arena.goal_width / 2.0),
                                                      (rules.arena.goal_width / 2.0) - rules.arena.bottom_radius),
                                                clamp(point.getY(),
                                                      rules.arena.bottom_radius,
                                                      rules.arena.goal_height - rules.arena.goal_top_radius),
                                                (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius),
                                           rules.arena.bottom_radius));
    }
    // Corner
    if (point.getX() > (rules.arena.width / 2.0) - rules.arena.corner_radius
        and point.getZ() > (rules.arena.depth / 2.0) - rules.arena.corner_radius) {
        dan = min(dan, dan_to_sphere_inner(
                                           point,
                                           Vec3(
                                                (rules.arena.width / 2.0) - rules.arena.corner_radius,
                                                point.getY(),
                                                (rules.arena.depth / 2.0) - rules.arena.corner_radius
                                                ),
                                           rules.arena.corner_radius));
    }
    // Goal outer corner
    if (point.getZ() < (rules.arena.depth / 2.0) + rules.arena.goal_side_radius) {
        // Side x
        if (point.getX() < (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius) {
            dan = min(dan, dan_to_sphere_outer(
                                               point,
                                               Vec3(
                                                    (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius,
                                                    point.getY(),
                                                    (rules.arena.depth / 2.0) + rules.arena.goal_side_radius
                                                    ),
                                               rules.arena.goal_side_radius));
            
        }
        // Ceiling
        if (point.getY() < rules.arena.goal_height + rules.arena.goal_side_radius) {
            dan = min(dan, dan_to_sphere_outer(
                                               point,
                                               Vec3(
                                                    point.getX(),
                                                    rules.arena.goal_height + rules.arena.goal_side_radius,
                                                    (rules.arena.depth / 2.0) + rules.arena.goal_side_radius
                                                    ),
                                               rules.arena.goal_side_radius));
        }
        // Top corner
        Vec3 o1 = Vec3((rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius,
                       rules.arena.goal_height - rules.arena.goal_top_radius, 0.0);
        Vec3 v = Vec3(point.getX(), point.getY(), 0.0) - o1;
        if (v.getX() > 0.0 && v.getY() > 0.0) {
            Vec3 o = o1 + v.normalized() * (rules.arena.goal_top_radius + rules.arena.goal_side_radius);
            dan = min(dan, dan_to_sphere_outer(
                                               point,
                                               Vec3(o.getX(), o.getY(), (rules.arena.depth / 2.0) + rules.arena.goal_side_radius),
                                               rules.arena.goal_side_radius));
            
        }
    }
    // Goal inside top corners
    if (point.getZ() > (rules.arena.depth / 2.0) + rules.arena.goal_side_radius
        && point.getY() > rules.arena.goal_height - rules.arena.goal_top_radius) {
        // Side x
        if (point.getX() > (rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                                               point,
                                               Vec3(
                                                    (rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius,
                                                    rules.arena.goal_height - rules.arena.goal_top_radius,
                                                    point.getZ()
                                                    ),
                                               rules.arena.goal_top_radius));
            
        }
        // Side z
        if (point.getZ() > (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.goal_top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                                               point,
                                               Vec3(
                                                    point.getX(),
                                                    rules.arena.goal_height - rules.arena.goal_top_radius,
                                                    (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.goal_top_radius
                                                    ),
                                               rules.arena.goal_top_radius));
        }
    }
    // Bottom corners
    if (point.getY() < rules.arena.bottom_radius) {
        // Side x
        if (point.getX() > (rules.arena.width / 2.0) - rules.arena.bottom_radius) {
            dan = min(dan, dan_to_sphere_inner(
                                               point,
                                               Vec3(
                                                    (rules.arena.width / 2.0) - rules.arena.bottom_radius,
                                                    rules.arena.bottom_radius,
                                                    point.getZ()
                                                    ),
                                               rules.arena.bottom_radius));
        }
        // Side z
        if (point.getZ() > (rules.arena.depth / 2.0) - rules.arena.bottom_radius
            and point.getX() >= (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius) {
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                    point.getX(),
                                                    rules.arena.bottom_radius,
                                                    (rules.arena.depth / 2.0) - rules.arena.bottom_radius
                                                    ),
                                               rules.arena.bottom_radius));
        }
        // Side z (goal)
        if (point.getZ() > (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius) {
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                    point.getX(),
                                                    rules.arena.bottom_radius,
                                                    (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius
                                                    ),
                                               rules.arena.bottom_radius));
        }
        // Goal outer corner
        Vec3 o1 = Vec3((rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius,
                       (rules.arena.depth / 2.0) + rules.arena.goal_side_radius, 0.0);
        Vec3 v = Vec3(point.getX(), point.getZ(), 0.0) - o1;
        if (v.getX() < 0.0 && v.getY() < 0.0 && v.len() < rules.arena.goal_side_radius + rules.arena.bottom_radius) {
            Vec3 o = o1 + v.normalized() * (rules.arena.goal_side_radius + rules.arena.bottom_radius);
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(o.getX(), rules.arena.bottom_radius, o.getY()),
                                               rules.arena.bottom_radius));
            
        }
        // Side x (goal)
        if (point.getZ() >= (rules.arena.depth / 2.0) + rules.arena.goal_side_radius
            and point.getX() > (rules.arena.goal_width / 2.0) - rules.arena.bottom_radius) {
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                    (rules.arena.goal_width / 2.0) - rules.arena.bottom_radius,
                                                    rules.arena.bottom_radius, point.getZ()),
                                               rules.arena.bottom_radius));
            
        }
        // Corner
        if (point.getX() > (rules.arena.width / 2.0) - rules.arena.corner_radius
            and point.getZ() > (rules.arena.depth / 2.0) - rules.arena.corner_radius) {
            Vec3 corner_o = Vec3((rules.arena.width / 2.0) - rules.arena.corner_radius,
                                 (rules.arena.depth / 2.0) - rules.arena.corner_radius, 0.0);
            Vec3 n = Vec3(point.getX(), point.getZ(), 0.0) - corner_o;
            double dist = n.len();
            if (dist > rules.arena.corner_radius - rules.arena.bottom_radius) {
                n = n / dist;
                Vec3 o2 = corner_o + n * (rules.arena.corner_radius - rules.arena.bottom_radius);
                dan = min(dan, dan_to_sphere_inner(
                                                   point,
                                                   Vec3(o2.getX(), rules.arena.bottom_radius, o2.getY()),
                                                   rules.arena.bottom_radius));
            }
        }
    }
    // Ceiling corners
    if (point.getY() > rules.arena.height - rules.arena.top_radius) {
        // Side x
        if (point.getX() > (rules.arena.width / 2.0) - rules.arena.top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                                               point,
                                               Vec3(
                                                    (rules.arena.width / 2.0) - rules.arena.top_radius,
                                                    rules.arena.height - rules.arena.top_radius,
                                                    point.getZ()
                                                    ),
                                               rules.arena.top_radius));
            
        }
        // Side z
        if (point.getZ() > (rules.arena.depth / 2.0) - rules.arena.top_radius) {
            dan = min(dan, dan_to_sphere_inner(
                                               point,
                                               Vec3(
                                                    point.getX(),
                                                    rules.arena.height - rules.arena.top_radius,
                                                    (rules.arena.depth / 2.0) - rules.arena.top_radius
                                                    ),
                                               rules.arena.top_radius));
            
        }
        
        // Corner
        if (point.getX() > (rules.arena.width / 2.0) - rules.arena.corner_radius
            && point.getZ() > (rules.arena.depth / 2.0) - rules.arena.corner_radius) {
            Vec3 corner_o = Vec3((rules.arena.width / 2.0) - rules.arena.corner_radius,
                                 (rules.arena.depth / 2.0) - rules.arena.corner_radius, 0.0);
            Vec3 dv = Vec3(point.getX(), point.getZ(), 0.0) - corner_o;
            if (dv.len() > rules.arena.corner_radius - rules.arena.top_radius) {
                Vec3 n = dv.normalized();
                Vec3 o2 = corner_o + n * (rules.arena.corner_radius - rules.arena.top_radius);
                dan = min(dan, dan_to_sphere_inner(
                                                   point,
                                                   Vec3(o2.getX(), rules.arena.height - rules.arena.top_radius, o2.getY()),
                                                   rules.arena.top_radius));
            }
        }
    }
    return dan;
}

Dan SimulationEngine::dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal) {
    return Dan(dot((point - point_on_plane), plane_normal), plane_normal);
}

Dan SimulationEngine::dan_to_sphere_inner(Vec3 point, Vec3 sphere_center, double sphere_radius) {
    return Dan(sphere_radius - (point - sphere_center).len(), (sphere_center - point).normalized());
}

Dan SimulationEngine::dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius) {
    return Dan((point - sphere_center).len() - sphere_radius, (point - sphere_center).normalized());
}

Dan SimulationEngine::min(Dan a, Dan b) {
    return a.distance < b.distance ? a : b;
}

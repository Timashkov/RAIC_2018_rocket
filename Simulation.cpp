//
//  Simulation.cpp
//  raic
//
//  Created by Alex on 23/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "Simulation.h"
#include "CVL_Utils.h"

Dan Simulation::dan_to_plane(Vec3 point, Vec3 point_on_plane, Vec3 plane_normal){
    return Dan(dot((point-point_on_plane), plane_normal), plane_normal);
}

Dan Simulation::dan_to_sphere_inner(Vec3 point, Vec3 sphere_center,double sphere_radius){
    return Dan(sphere_radius - (point - sphere_center).len(), (sphere_center - point).normalized());
}

Dan Simulation::dan_to_sphere_outer(Vec3 point, Vec3 sphere_center, double sphere_radius){
    return Dan((point-sphere_center).len() - sphere_radius, (point-sphere_center).normalized());
}

Dan Simulation::min(Dan a, Dan b){
    return a.distance < b.distance? a : b;
}

Dan Simulation::dan_to_arena_quarter(Vec3 point){    // Ground
    Dan dan = dan_to_plane(point, Vec3(0.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
    
    // Ceiling
    dan = min(dan, dan_to_plane(point, Vec3(0.0, arena.height, 0.0), Vec3(0.0, -1.0, 0.0)));
    
    // Side x
    dan = min(dan, dan_to_plane(point, Vec3(arena.width / 2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
    
    // Side z (goal)
    dan = min(dan, dan_to_plane(point, Vec3(0.0, 0.0, (arena.depth / 2.0) + arena.goal_depth), Vec3(0.0, 0.0, -1.0)));
    
    // Side z
    Vec3 v = Vec3(point.getX(), point.getY(),0.0) - (Vec3((arena.goal_width / 2.0) - arena.goal_top_radius,arena.goal_height - arena.goal_top_radius,0.0));
    if (point.getX() >= (arena.goal_width / 2.0) + arena.goal_side_radius
        || point.getY() >= arena.goal_height + arena.goal_side_radius || (
                                                                          v.getX() > 0.0
                                                                          && v.getY() > 0.0
                                                                          && v.len() >= arena.goal_top_radius + arena.goal_side_radius)){
            dan = min(dan, dan_to_plane(point, Vec3(0.0, 0.0, arena.depth / 2.0), Vec3(0.0, 0.0, -1.0)));
        }
    
    // Side x & ceiling (goal)
    if (point.getZ() >= (arena.depth / 2.0) + arena.goal_side_radius){
        // x
        dan = min(dan, dan_to_plane(point, Vec3(arena.goal_width / 2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
        // y
        dan = min(dan, dan_to_plane(point, Vec3(0.0, arena.goal_height, 0.0), Vec3(0.0, -1.0, 0.0)));
    }
    
    // Goal back corners
    //    assert arena.bottom_radius == arena.goal_top_radius
    if (point.getZ() > (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius){
        dan = min(dan, dan_to_sphere_inner(point,
                                           Vec3(clamp(point.getX(), arena.bottom_radius - (arena.goal_width / 2.0),(arena.goal_width / 2.0)- arena.bottom_radius),
                                                clamp(point.getY(),
                                                      arena.bottom_radius,
                                                      arena.goal_height - arena.goal_top_radius),
                                                (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius),
                                           arena.bottom_radius));
    }
    // Corner
    if (point.getX() > (arena.width / 2.0) - arena.corner_radius
        and point.getZ() > (arena.depth / 2.0) - arena.corner_radius){
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
    if (point.getZ() < (arena.depth / 2.0) + arena.goal_side_radius){
        // Side x
        if (point.getX() < (arena.goal_width / 2.0) + arena.goal_side_radius){
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
        if (point.getY() < arena.goal_height + arena.goal_side_radius){
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
        Vec3 v = Vec3(point.getX(), point.getY(),0.0) - o1;
        if (v.getX() > 0.0 && v.getY() > 0.0){
            Vec3 o = o1 + v.normalized() * (arena.goal_top_radius + arena.goal_side_radius);
            dan = min(dan, dan_to_sphere_outer(
                                               point,
                                               Vec3(o.getX(), o.getY(), (arena.depth / 2.0) + arena.goal_side_radius),
                                               arena.goal_side_radius));
            
        }
    }
    // Goal inside top corners
    if (point.getZ() > (arena.depth / 2.0) + arena.goal_side_radius
        && point.getY() > arena.goal_height - arena.goal_top_radius){
        // Side x
        if (point.getX() > (arena.goal_width / 2.0) - arena.goal_top_radius){
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
        if (point.getZ() > (arena.depth / 2.0) + arena.goal_depth - arena.goal_top_radius){
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
    if (point.getY() < arena.bottom_radius){
        // Side x
        if (point.getX() > (arena.width / 2.0) - arena.bottom_radius){
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
            and point.getX() >= (arena.goal_width / 2.0) + arena.goal_side_radius){
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                    point.getX(),
                                                    arena.bottom_radius,
                                                    (arena.depth / 2.0) - arena.bottom_radius
                                                    ),
                                               arena.bottom_radius));
        }
        // Side z (goal)
        if (point.getZ() > (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius){
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
                       (arena.depth / 2.0) + arena.goal_side_radius,0.0);
        Vec3 v = Vec3(point.getX(), point.getZ(),0.0) - o1;
        if (v.getX() < 0.0 && v.getY() < 0.0 && v.len() < arena.goal_side_radius + arena.bottom_radius){
            Vec3 o = o1 + v.normalized() * (arena.goal_side_radius + arena.bottom_radius);
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(o.getX(), arena.bottom_radius, o.getY()),
                                               arena.bottom_radius));
            
        }
        // Side x (goal)
        if (point.getZ() >= (arena.depth / 2.0) + arena.goal_side_radius
            and point.getX() > (arena.goal_width / 2.0) - arena.bottom_radius){
            dan = min(dan, dan_to_sphere_inner(point,
                                               Vec3(
                                                    (arena.goal_width / 2.0) - arena.bottom_radius,
                                                    arena.bottom_radius, point.getZ()),
                                               arena.bottom_radius));
            
        }
        // Corner
        if (point.getX() > (arena.width / 2.0) - arena.corner_radius
            and point.getZ() > (arena.depth / 2.0) - arena.corner_radius){
            Vec3 corner_o = Vec3((arena.width / 2.0) - arena.corner_radius,
                                 (arena.depth / 2.0) - arena.corner_radius, 0.0 );
            Vec3 n = Vec3(point.getX(), point.getZ(),0.0) - corner_o;
            double dist = n.len();
            if (dist > arena.corner_radius - arena.bottom_radius){
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
    if (point.getY() > arena.height - arena.top_radius){
        // Side x
        if (point.getX() > (arena.width / 2.0) - arena.top_radius){
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
        if (point.getZ() > (arena.depth / 2.0) - arena.top_radius){
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
            && point.getZ() > (arena.depth / 2.0) - arena.corner_radius){
            Vec3 corner_o = Vec3( (arena.width / 2.0) - arena.corner_radius,
                                 (arena.depth / 2.0) - arena.corner_radius, 0.0);
            Vec3 dv = Vec3(point.getX(), point.getZ(),0.0) - corner_o;
            if (dv.len() > arena.corner_radius - arena.top_radius){
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

Dan Simulation::dan_to_arena(Vec3& point){
    bool negate_x = point.getX() < 0.0;
    bool negate_z = point.getZ() < 0.0;
    if (negate_x){
        point.setX(-point.getX());
    }
    if (negate_z){
        point.setZ(-point.getZ());
    }
    Dan result = dan_to_arena_quarter(point);
    if (negate_x){
        result.normal.setX(-result.normal.getX());
    }
    if (negate_z){
        result.normal.setZ(-result.normal.getZ());
    }
    return result;
}

void Simulation::collide_entities(Entity& a,Entity& b){
    Vec3 delta_position = b.position-a.position;
    double distance = delta_position.len();
    double penetration = a.radius + b.radius - distance;
    if (penetration > 0.0){
        double k_a = (1.0 / a.mass) / ((1.0/ a.mass) + (1.0 / b.mass));
        double k_b = (1.0 / b.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
        Vec3 normal = delta_position.normalized();
        a.setPosition(a.position- (normal * penetration * k_a));
        b.setPosition(b.position + (normal * penetration * k_b));
        Vec3 velodelta = b.velocity-a.velocity;
        double delta_velocity = dot(velodelta, normal) + b.radius_change_speed - a.radius_change_speed;
        if (delta_velocity < 0.0){
            Vec3 impulse = normal * (1.0 + random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity;
            a.velocity = a.velocity + (impulse * k_a);
            b.velocity = b.velocity - (impulse * k_b);
        }
    }
}

Vec3 Simulation::collide_with_arena(Entity& e){
    Dan danArena = dan_to_arena(e.position);
    double penetration = e.radius - danArena.distance;
    if (penetration > 0.0){
        e.position = e.position + danArena.normal * penetration;
        double velocity = dot(e.velocity, danArena.normal) - e.radius_change_speed;
        if (velocity < 0.0){
            e.velocity = e.velocity - danArena.normal * velocity * (1.0 + rules.BALL_ARENA_E);
            return danArena.normal;
        }
    }
    return Vec3::None;
}

void Simulation::move(Entity& e, double delta_time){
    e.velocity = clamp(e.velocity, rules.MAX_ENTITY_SPEED);
    e.position = e.position + e.velocity * delta_time;
    e.position.setY(e.position.getY() - rules.GRAVITY * delta_time * delta_time / 2.0);
    e.velocity.setY(e.velocity.getY() - rules.GRAVITY * delta_time);
}

void Simulation::update(float delta_time){
    // FIXME: after investigations
    //    shuffle(robots);
    
    for (Entity robot : robots){
        if (robot.touch){
            Vec3 target_velocity = clamp(robot.action_target_velocity,rules.ROBOT_MAX_GROUND_SPEED);
            //wtf touch_normal??
            target_velocity = target_velocity - (robot.touch_normal * dot(robot.touch_normal, target_velocity));
            Vec3 target_velocity_change = target_velocity - robot.velocity;
            if (target_velocity_change.len() > 0.0){
                double acceleration = rules.ROBOT_ACCELERATION * max(0.0, robot.touch_normal.getY());
                robot.velocity = robot.velocity + clamp(target_velocity_change.normalized() * acceleration * delta_time, target_velocity_change.len());
            }
        }
        
        if (robot.action_use_nitro){
            Vec3 target_velocity_change = clamp(robot.action_target_velocity - robot.velocity,robot.nitro * rules.NITRO_POINT_VELOCITY_CHANGE);
            if (target_velocity_change.len() > 0.0){
                Vec3 acceleration = target_velocity_change.normalized() * rules.ROBOT_NITRO_ACCELERATION;
                Vec3 velocity_change = clamp(acceleration * delta_time,target_velocity_change.len());
                robot.velocity = robot.velocity + velocity_change;
                robot.nitro = robot.nitro - velocity_change.len()/ rules.NITRO_POINT_VELOCITY_CHANGE;
            }
        }
        move(robot, delta_time);
        robot.radius = rules.ROBOT_MIN_RADIUS + (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action_jump_speed / rules.ROBOT_MAX_JUMP_SPEED;
        robot.radius_change_speed = robot.action_jump_speed;
    }
    move(ball, delta_time);
    
    for (int i = 0; i< robots.size(); i++ ){
        for (int j = 0; j < i - 1; j++){
            collide_entities(robots[i], robots[j]);
        }
    }
    
    for ( Entity robot : robots){
        collide_entities(robot, ball);
        Vec3 collision_normal = collide_with_arena(robot);
        if (collision_normal == Vec3::None){
            robot.touch = false;
        } else {
            robot.touch = true;
            robot.touch_normal = collision_normal;
        }
    }
    collide_with_arena(ball);
    
    if (abs(ball.position.getZ()) > arena.depth / 2.0 + ball.radius){
        goal_scored();
    }
    
    for (Entity robot : robots){
        if (robot.nitro == rules.MAX_NITRO_AMOUNT)
            continue;
        for (Entity pack : nitro_packs){
            if (!pack.alive)
                continue;
            if ((robot.position - pack.position).len() <= robot.radius + pack.radius){
                robot.nitro = rules.MAX_NITRO_AMOUNT;
                pack.alive = false;
                pack.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS;
            }
        }
    }
}

void Simulation::tick(){
    float delta_time = 1.0 / rules.TICKS_PER_SECOND;
    for (int i = 0; i< rules.MICROTICKS_PER_TICK - 1; i++){
        update(delta_time / rules.MICROTICKS_PER_TICK);
    }
    
    for (Entity pack : nitro_packs){
        if (pack.alive){
            continue;
        }
        pack.respawn_ticks -= 1;
        if (pack.respawn_ticks == 0)
            pack.alive = true;
    }
    
   
}

void Simulation::go(){
    for (int i =0 ; i < 20; i++){
    tick();
    std::stringstream ss;
        ss << " simulaton:: tick : " << current_tick << endl;
    ss << " BALL radius:" << ball.radius;
    ss << " coord:(" << ball.position.getX() << ";" << ball.position.getY() << ";" << ball.position.getZ() << ")";
    ss << " velocity:(" << ball.velocity.getX() << ";" << ball.velocity.getY() << ";" << ball.velocity.getZ() << ")" << std::endl;
    
    for (Entity r: robots){
        ss << " ROBOT: id: *" << r.id << "* " << "* ";
        ss << " player_id: *" << r.player_id << "*";
        ss << " coord: (" << r.position.getX() << ";" << r.position.getY() << ":" << r.position.getZ() << ")";
        ss << " velocity: (" << r.velocity.getX() << ";" << r.velocity.getY() << ";" << r.velocity.getZ() << ")";
        ss << " radius:" << r.radius;
        ss << " nitro:" << r.nitro;
        ss << " touch:" << r.touch;
        ss << " touch_normal: x:" << r.touch_normal.getX() << "Y:" << r.touch_normal.getY() << "z:" << r.touch_normal.getZ() << std::endl<<std::endl;
    }
    
    writeLog(ss);
        current_tick++;
        
    }
}

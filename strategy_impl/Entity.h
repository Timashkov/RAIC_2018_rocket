//
// Created by timashkov on 12/18/18.
//

#ifndef STRATEGY_ENTITY_H
#define STRATEGY_ENTITY_H

#include "VecUtils.h"

class Entity{
protected:
    Vec2 position;
    Vec2 velocity;
public:
    Entity(){}
    ~Entity(){}

};


class Ball: protected Entity{

};

class Robot: protected Entity{

};


#endif //STRATEGY_ENTITY_H

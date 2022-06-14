/**
 * @file position.h
 * @author Mathis Lécrivain
 * @brief Position structure
 * @date 2022-06-14 (Creation)
 * 
 * @copyright (c) 2022
 * 
 */
#ifndef INC_POSITION_H
#define INC_POSITION_H

struct Position {
    Position()
        : x(0)
        , y(0)
        , theta(0)
    {
    }
    Position(float x, float y, float theta)
        : x(x)
        , y(y)
        , theta(theta)
    {
    }

    float x, y, theta;
};

#endif // INC_POSITION_H

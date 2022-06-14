/**
 * @file abstract_odometry.h
 * @author Mathis Lécrivain
 * @brief Odometry abstraction object
 * @date 2022-06-09 (Creation)
 * 
 * @copyright (c) 2022
 * 
 */
#ifndef INC_ABSTRACT_ODOMETRY_H
#define INC_ABSTRACT_ODOMETRY_H

#include "position.h"

class AbstractOdometry {
public:
    virtual ~AbstractOdometry() { }

    virtual Position get_position() const = 0;
    virtual float get_lin_velocity() const = 0;
    virtual float get_ang_velocity() const = 0;
};

#endif // INC_ABSTRACT_ODOMETRY_H

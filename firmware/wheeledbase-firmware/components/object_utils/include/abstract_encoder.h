/**
 * @file abstract_encoder.h
 * @author Mathis Lécrivain
 * @brief Encoder abstraction object
 * @date 2022-06-09 (Creation)
 * 
 * @copyright (c) 2022
 * 
 */
#ifndef INC_ABSTRACT_ENCODER_H
#define INC_ABSTRACT_ENCODER_H

class AbstractEncoder {
public:
    virtual ~AbstractEncoder() { }

    virtual float get_traveled_distance() = 0;
    virtual float restart() = 0;
};

#endif // INC_ABSTRACT_ENCODER_H

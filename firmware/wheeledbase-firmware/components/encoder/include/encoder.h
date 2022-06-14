/**
 * @file encoder.h
 * @author Mathis Lécrivain
 * @brief Encoder object
 * @date 2022-06-07 (Creation)
 *
 * @copyright (c) 2022
 *
 */
#ifndef INC_ENCODER_H
#define INC_ENCODER_H

#include <math.h>

#include "driver/gpio.h"
#include "driver/pcnt.h"

#include "abstract_encoder.h"

class Encoder : public AbstractEncoder {
public:
    Encoder(const char* name)
        : _current_counter(0)
        , _start_counter(0)
        , _wheel_radius(1 / (2 * M_PI))
        , _counts_per_rev(1000)
    {
        _name = name;
    }

    /**
     * @brief Attach PCNT counter
     * 
     * @param unit 
     * @param phase_a_gpio 
     * @param phase_b_gpio 
     */
    void attach_counter(pcnt_unit_t unit, gpio_num_t phase_a_gpio, gpio_num_t phase_b_gpio);

    /**
     * @brief Get the counter value
     * 
     * @return int64_t 
     */
    int64_t get_counter(void);

    /**
     * @brief Get the counts per revolution
     * 
     * @return int64_t 
     */
    int64_t get_counts_per_revolution(void) { return _counts_per_rev; }

    /**
     * @brief Get the wheel radius
     * 
     * @return float 
     */
    float get_wheel_radius(void) { return _wheel_radius; }

    /**
     * @brief Get the pcnt unit
     * 
     * @return pcnt_unit_t 
     */
    pcnt_unit_t get_pcnt_unit(void) { return _unit; }

    /**
     * @brief Set the counts per revolution
     * 
     * @param value 
     */
    void set_counts_per_revolution(int64_t value);

    /**
     * @brief Set the wheel radius object
     * 
     * @param value 
     */
    void set_wheel_radius(float value);

    /**
     * @brief add value to overflow counter
     * 
     * @param value 
     */
    void overflow_counter_add(int64_t value) { _overflow_counter = value; }

    /**
     * @brief reset counter values
     * 
     */
    void reset(void);

    /**
     * @brief Get the traveled distance in mm
     * 
     * @return float 
     */
    float get_traveled_distance(void);

    /**
     * @brief restart counter
     * 
     * @return float 
     */
    float restart(void);

    /**
     * @brief Load configuration form NVS
     *
     */
    void load_config(void);

    /**
     * @brief Save configuration to NVS
     *
     */
    void save_config(void) const;

private:
    const char* _name;

protected:
    int64_t _current_counter; /* Current tick counter */
    int64_t _start_counter; /* Tick counter from last reset */
    int64_t _overflow_counter; /* PCNT overflow tick counter */

    float _wheel_radius; /* Wheel radius in mm */
    int64_t _counts_per_rev; /* Counts per wheel revolution */

    pcnt_unit_t _unit; /* PCNT unit */
};

#endif // INC_ENCODER_H

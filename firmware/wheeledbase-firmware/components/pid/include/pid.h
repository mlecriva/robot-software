/**
 * @file pid.h
 * @author Mathis Lécrivain
 * @brief PID Corrector
 * @date 2022-06-14 (Creation)
 *
 * @copyright (c) 2022
 *
 */

#ifndef INC_PID_H
#define INC_PID_H

#include <math.h>

class PID {
public:
    PID(const char* name)
        : _Kp(1)
        , _Ki(0)
        , _Kd(0)
        , _min_output(-INFINITY)
        , _max_output(INFINITY)
    {
        _name = name;
    }
    /**
     * @brief Compute asservissement output
     *
     * @param setpoint Desired value
     * @param input Real value.
     * @param timestep time between last call.
     *
     * @return float
     */
    float compute(float setpoint, float input, float timestep);

    /**
     * @brief Reset accumulators
     */
    void reset(void);

    /**
     * @brief Set new tunning constants

     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    void set_tunings(float Kp, float Ki, float Kd);

    /**
     * @brief Charge les limites de sorties.
     *
     *
     * @param min_output Minimum output (Can be negative).
     * @param max_output Maximum output.
     */
    void set_output_limits(float min_output, float max_output);

    /**
     * @brief Get the Kp coefficient
     *
     * @return float
     */
    float get_Kp(void) const
    {
        return _Kp;
    }

    /**
     * @brief Get the Ki coefficient
     *
     * @return float
     */
    float get_Ki(void) const
    {
        return _Ki;
    }

    /**
     * @brief Get the Kd coefficient
     *
     * @return float
     */
    float get_Kd(void) const
    {
        return _Kd;
    }

    /**
     * @brief Get the min output value
     *
     * @return float
     */
    float get_min_output(void) const
    {
        return _min_output;
    }

    /**
     * @brief Get the max output value
     *
     * @return float
     */
    float get_max_output(void) const
    {
        return _max_output;
    }

    /**
     * @brief Load parameters from NVS memory
     */
    void load_config(void);

    /**
     * @brief Save parameter to NVS memory
     */
    void save_config(void) const;

private:
    const char* _name;

    float _error_integral; /* Integral accumulator */
    float _previous_error; /* previous error */

    float _Kp; /* Proportional coefficient */
    float _Ki; /* Integral coefficient */
    float _Kd; /* Derivative coefficient */
    float _min_output; /* Minimal output */
    float _max_output; /* Maximal output */
};

#endif // INC_PID_H

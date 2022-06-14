/**
 * @file odometry.h
 * @author Mathis Lécrivain
 * @brief Odometry calculation object
 * @date 2022-06-14 (Creation)
 * 
 * @copyright  (c) 2022
 * 
 */

#ifndef INC_ODOMETRY_H
#define INC_ODOMETRY_H

#include "abstract_encoder.h"
#include "abstract_odometry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "position.h"

#define ODOMETRY_DEFAULT_TIMESTEP (5) /* ms */

class Odometry : public AbstractOdometry {
public:
    Odometry(const char* name)
        : _timestep(ODOMETRY_DEFAULT_TIMESTEP)
    {
        _name = name;
    }

    /**
     * @brief Start odometry process loop
     * 
     */
    void start(void);

    /**
     * @brief Set the default odometry position 
     * 
     * @param x X coordinate (mm)
     * @param y Y coordinate (mm)
     * @param theta angle (rad)
     */
    void set_position(float x, float y, float theta)
    {
        _pos.x = x;
        _pos.y = y;
        _pos.theta = theta;
    }

    /**
     * @brief Set the axle track dimension
     * 
     * @param axle_track Axle track value (mm)
     */
    void set_axle_track(float axle_track) { _axle_track = axle_track; }

    /**
     * @brief Set the slippage value
     * 
     * @param slippage Slippage value
     */
    void set_slippage(float slippage) { _slippage = slippage; }

    /**
     * @brief Set the encoders objects
     * 
     * @param left_encoder Left encoder
     * @param right_encoder Right encoder
     */
    void set_encoders(AbstractEncoder& left_encoder, AbstractEncoder& right_encoder)
    {
        _left_encoder = &left_encoder, _right_encoder = &right_encoder;
    }

    /**
     * @brief Get the odometry position object
     * 
     * @return const Position 
     */
    Position get_position() const { return _pos; }

    /**
     * @brief Get the linear velocity value
     * 
     * @return float linear velocity (mm/s)
     */
    float get_lin_velocity() const { return _lin_velocity; }

    /**
     * @brief Get the angular velocity value
     * 
     * @return float angular velocity (rang/s)
     */
    float get_ang_velocity() const { return _ang_velocity; }

    /**
     * @brief Get the axle track value
     * 
     * @return float Axle track value (mm)
     */
    float get_axle_track() const { return _axle_track; }

    /**
     * @brief Get the slippage value
     * 
     * @return float slippage value
     */
    float get_slippage() const { return _slippage; }

    /**
     * @brief Task function to periodically compute odometry
     * 
     */
    void task(void);

    /**
     * @brief Load configuration form NVS
     *
     */
    void load_config(void);

    /**
     * @brief  save configuration to NVS
     *
     */
    void save_config(void) const;

private:
    const char* _name;

    Position _pos;
    float _lin_velocity;
    float _ang_velocity;
    float _axle_track;
    float _slippage;
    uint32_t _timestep;

    AbstractEncoder* _left_encoder;
    AbstractEncoder* _right_encoder;

    TaskHandle_t _task_handle;
};

#endif /* INC_ODOMETRY_H */

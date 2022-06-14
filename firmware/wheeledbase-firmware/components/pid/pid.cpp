/**
 * @file pid.cpp
 * @author Mathis Lécrivain
 * @brief PID Corrector
 * @date 2022-06-14 (Creation)
 * 
 * @copyright (c) 2022
 * 
 */
#include "pid.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"

/****************************************************************************************************
 *                                         Private defines
 ***************************************************************************************************/
#define NVS_PARTITION_NAME "storage"

/****************************************************************************************************
 *                                       Private structures
 ***************************************************************************************************/
typedef struct {
    float Kp; /* Proportional coefficient */
    float Ki; /* Integral coefficient */
    float Kd; /* Derivative coefficient */
    float min_output; /* Minimal output */
    float max_output; /* Maximal output */
} pid_nvs_config_t;

/****************************************************************************************************
 *                                        Private variables
 ***************************************************************************************************/

/****************************************************************************************************
 *                                     Private function definition
 ***************************************************************************************************/
/**
 * @brief Saturate value between min and max
 * 
 * @param x Value to saturate
 * @param min Min value
 * @param max Max value
 * @return float 
 */
static float saturate(float x, float min, float max)
{
    if (x < min)
        return min;
    else if (x > max)
        return max;
    else
        return x;
}

/****************************************************************************************************
 *                                     Class function definition
 ***************************************************************************************************/
float PID::compute(float setpoint, float input, float timestep)
{
    /* Compute the error between the current state and the setpoint */
    float current_error = setpoint - input;

    /* Compute the error integral */
    _error_integral += current_error * timestep;
    _error_integral = saturate(_error_integral, _min_output / _Ki, _max_output / _Ki);

    /* Compute the derivative error */
    float error_derivative = (current_error - _previous_error) / timestep;
    _previous_error = current_error;

    /* Compute the PID controller's output */
    float output = _Kp * current_error + _Ki * _error_integral - _Kd * error_derivative;
    return saturate(output, _min_output, _max_output);
}

void PID::reset(void)
{
    _error_integral = 0;
    _previous_error = 0;
}

void PID::set_tunings(float Kp, float Ki, float Kd)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PID::set_output_limits(float min_output, float max_output)
{
    _min_output = min_output;
    _max_output = max_output;
}

void PID::load_config(void)
{
    std::shared_ptr<nvs::NVSHandle> handle;
    esp_err_t result;
    pid_nvs_config_t config;

    /* Handle will automatically close when going out of scope or when it's reset. */
    handle = nvs::open_nvs_handle(NVS_PARTITION_NAME, NVS_READWRITE, &result);

    if (result != ESP_OK) {
        ESP_LOGE(_name, "Error (%s) opening NVS handle!", esp_err_to_name(result));
    } else {
        ESP_LOGI(_name, "Reading configuration from NVS ... ");

        result = handle->get_blob(_name, (void*)&config, sizeof(config));
        switch (result) {
        case ESP_OK:
            _Kp = config.Kp;
            _Ki = config.Ki;
            _Kd = config.Kd;

            _min_output = config.min_output;
            _max_output = config.max_output;

            ESP_LOGI(_name, "Loaded configuration :"
                            "\n\t -> Kp : %f"
                            "\n\t -> Ki : %f"
                            "\n\t -> Kd : %f"
                            "\n\t -> min_output : %f"
                            "\n\t -> max_output : %f",
                _Kp, _Ki, _Kd, _min_output, _max_output);

            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(_name, "Configuration is not initialized yet!");
            break;
        default:
            ESP_LOGE(_name, "Error (%s) reading!", esp_err_to_name(result));
        }
    }
}

void PID::save_config(void) const
{
    std::shared_ptr<nvs::NVSHandle> handle;
    esp_err_t result;

    pid_nvs_config_t config = {
        .Kp = _Kp,
        .Ki = _Ki,
        .Kd = _Kd,
        .min_output = _min_output,
        .max_output = _max_output,
    };

    /* Handle will automatically close when going out of scope or when it's reset. */
    handle = nvs::open_nvs_handle(NVS_PARTITION_NAME, NVS_READWRITE, &result);

    if (result != ESP_OK) {
        ESP_LOGE(_name, "Error (%s) opening NVS handle!", esp_err_to_name(result));
    } else {
        ESP_LOGI(_name, "Updating configuration in NVS ... ");
        result = handle->set_blob(_name, (const void*)&config, sizeof(config));
        if (result != ESP_OK) {
            ESP_LOGE(_name, "Failed!");
        }

        ESP_LOGI(_name, "Committing updates in NVS ... ");
        result = handle->commit();
        if (result != ESP_OK) {
            ESP_LOGE(_name, "Failed!");
        } else {
            ESP_LOGI(_name, "Done!");
        }
    }
}
